using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace FogOfWar.Engine;

/// <summary>
///     A <b>runtime-tiered, N-wide</b> bounding-volume hierarchy over the SAME static world-space triangle soup
///     as the scalar <see cref="Bvh" />, traversed with a <b>SIMD ray-vs-N-child-AABB slab test</b> (one op
///     tests all N children of a node at once). The lane width is chosen once at build time from the widest
///     instruction set the CPU exposes:
///     <list type="bullet">
///       <item><see cref="Tier.Avx512" /> — <see cref="Vector512{T}" />, 16 children/op (BVH16).</item>
///       <item><see cref="Tier.Avx2" /> — <see cref="Vector256{T}" />, 8 children/op (BVH8).</item>
///       <item><see cref="Tier.Sse" /> — <see cref="Vector128{T}" />, 4 children/op (BVH4).</item>
///     </list>
///     When no SIMD is available the caller keeps the scalar <see cref="Bvh" /> (also the correctness reference).
///
///     <para><b>Bit-for-bit identical boolean answers to the scalar BVH (0 mismatches, every tier).</b> SIMD
///     accelerates only BVH TRAVERSAL (which nodes to descend). The actual ray/triangle intersection at leaves
///     reuses the EXACT scalar Möller–Trumbore (<see cref="RayTriangle" />) — plain <see cref="Vector3" />
///     float math, <b>no FMA</b> — so every per-triangle decision is arithmetically the same as the scalar
///     reference. (FMA in the leaf math would change rounding and could flip a near-edge triangle result vs
///     scalar → a gate mismatch; leaves hold ≤ <c>Bvh.MaxLeafTris</c> triangles, so the traversal, not the leaf,
///     is the hot part — scalar leaves cost almost nothing and buy provable correctness.)</para>
///
///     <para><b>Why traversal can't leak.</b> The scalar BVH already matches an all-triangles brute force, so it
///     reports occluded iff SOME triangle is truly hit. This engine (1) uses the identical per-triangle test,
///     and (2) its slab test is <i>conservative</i>: it is the NEGATION of the miss condition, and any lane with
///     a NaN operand (0·∞ from an axis-aligned ray grazing a slab plane) is force-kept as HIT via an explicit
///     finite-operand check — never a spurious skip. A node whose AABB bounds a truly-hit triangle is therefore
///     always descended, so the leaf test finds the same hit scalar would ⇒ same boolean, every ray, every tier.</para>
///
///     <para><b>Immutable after construction → thread-safe.</b> <see cref="Occluded" /> only reads the node /
///     triangle arrays and uses a stack-local scratch stack (stackalloc, heap-grown on overflow), so a single
///     per-server worker thread (or many) may call it with no shared mutable state.</para>
/// </summary>
internal sealed class BvhSimd
{
    /// <summary>SIMD tier; the enum value is the node/vector lane width.</summary>
    public enum Tier
    {
        Sse    = 4,
        Avx2   = 8,
        Avx512 = 16,
    }

    // Geometry — shared (same references) with the scalar Bvh; original triangle order.
    private readonly Vector3[] _v0;
    private readonly Vector3[] _v1;
    private readonly Vector3[] _v2;
    private readonly int[]     _triIdx;

    // Node pool, struct-of-arrays. Each node owns a contiguous run of _width child slots at [node*w .. node*w+w-1].
    // For each slot c: the child AABB (min/max per axis) and a descriptor:
    //   _childCount[slot] > 0  → LEAF:     _childIdx[slot] is the first index into _triIdx, count triangles.
    //   _childCount[slot] == 0 → INTERNAL: _childIdx[slot] is the child node index.
    // Only the low _width bits of _validMask[node] mark real children; unused slots are ignored via the mask
    // (their AABB is arbitrary — masking, not an "empty" sentinel box, gates them, which is robust to the slab
    // test's min/max re-sorting that would otherwise resurrect an inverted "empty" box).
    private readonly float[] _minX;
    private readonly float[] _minY;
    private readonly float[] _minZ;
    private readonly float[] _maxX;
    private readonly float[] _maxY;
    private readonly float[] _maxZ;
    private readonly int[]   _childIdx;
    private readonly int[]   _childCount;
    private readonly int[]   _validMask; // up to 16 valid-child bits per node

    private readonly int  _nodeCount;
    private readonly int  _width;
    private readonly Tier _tier;

    private const float RayEps = 1e-4f; // identical to scalar Bvh.RayEps.

    private BvhSimd(
        Vector3[] v0, Vector3[] v1, Vector3[] v2, int[] triIdx,
        float[] minX, float[] minY, float[] minZ, float[] maxX, float[] maxY, float[] maxZ,
        int[] childIdx, int[] childCount, int[] validMask, int nodeCount, Tier tier)
    {
        _v0         = v0;
        _v1         = v1;
        _v2         = v2;
        _triIdx     = triIdx;
        _minX       = minX;
        _minY       = minY;
        _minZ       = minZ;
        _maxX       = maxX;
        _maxY       = maxY;
        _maxZ       = maxZ;
        _childIdx   = childIdx;
        _childCount = childCount;
        _validMask  = validMask;
        _nodeCount  = nodeCount;
        _width      = (int) tier;
        _tier       = tier;
    }

    public int  NodeCount     => _nodeCount;
    public int  TriangleCount => _v0.Length;
    public Tier ActiveTier    => _tier;
    public int  Width         => _width;

    /// <summary>True when at least one SIMD tier can run (SSE4.1 ⇒ SSE/SSE2 present); universal on x64.</summary>
    public static bool IsSupported => Sse41.IsSupported;

    /// <summary>
    ///     The widest tier this CPU runs NATIVELY. Gated on <c>Vector512/256.IsHardwareAccelerated</c>, not just
    ///     <c>Avx512F.IsSupported</c>: on Intel parts where the runtime caps 512-bit ops (PreferredVectorBitWidth
    ///     256, avoids license downclock) a Vector512 op is software-expanded to 2×256 — correct but slower than
    ///     the native AVX2 tier it would skip. Picking the hardware-accelerated width avoids that.
    /// </summary>
    public static Tier BestTier()
        => Avx512F.IsSupported && Vector512.IsHardwareAccelerated ? Tier.Avx512
         : Avx2.IsSupported   && Vector256.IsHardwareAccelerated ? Tier.Avx2
         : Tier.Sse;

    /// <summary>True if the given tier can run on this CPU.</summary>
    public static bool TierSupported(Tier tier) => tier switch
    {
        Tier.Avx512 => Avx512F.IsSupported,
        Tier.Avx2   => Avx2.IsSupported,
        Tier.Sse    => Sse41.IsSupported,
        _           => false,
    };

    // ── Build: widen the scalar binary BVH into an N-wide BVH ─────────────────────────────────────────────

    /// <summary>Build at the widest tier this CPU supports.</summary>
    public static BvhSimd Build(Bvh binary) => Build(binary, BestTier());

    /// <summary>
    ///     Build an N-wide BVH (N = the tier's lane width) by collapsing an already-built scalar
    ///     <see cref="Bvh" /> — reusing its validated binned-SAH partition and its triangle arrays (geometry is
    ///     SHARED, not copied). Starting from a binary node's two children, the internal child with the largest
    ///     surface area is repeatedly expanded into ITS two children until N children are collected or none
    ///     remain internal.
    /// </summary>
    public static BvhSimd Build(Bvh binary, Tier tier)
    {
        var width  = (int) tier;
        var v0     = binary.V0;
        var v1     = binary.V1;
        var v2     = binary.V2;
        var triIdx = binary.TriIdx;
        var nodes  = binary.Nodes;

        // Degenerate empty soup: no nodes; Occluded short-circuits on the triangle-count guard.
        if (v0.Length == 0)
        {
            return new BvhSimd(
                v0, v1, v2, triIdx,
                Array.Empty<float>(), Array.Empty<float>(), Array.Empty<float>(),
                Array.Empty<float>(), Array.Empty<float>(), Array.Empty<float>(),
                Array.Empty<int>(), Array.Empty<int>(), Array.Empty<int>(), 0, tier);
        }

        var minX = new List<float>();
        var minY = new List<float>();
        var minZ = new List<float>();
        var maxX = new List<float>();
        var maxY = new List<float>();
        var maxZ = new List<float>();
        var childIdx   = new List<int>();
        var childCount = new List<int>();
        var validMask  = new List<int>();

        int AllocWide()
        {
            var id = validMask.Count;
            for (var c = 0; c < width; c++)
            {
                minX.Add(0f); minY.Add(0f); minZ.Add(0f);
                maxX.Add(0f); maxY.Add(0f); maxZ.Add(0f);
                childIdx.Add(0);
                childCount.Add(-1); // invalid slot
            }
            validMask.Add(0);
            return id;
        }

        var queue    = new Queue<(int binIdx, int wideId)>();
        var children = new List<int>(width);

        var rootWide = AllocWide();

        if (nodes[0].Count > 0)
        {
            // Whole map is a single leaf (≤ MaxLeafTris). One wide node with one leaf child = the root.
            SetLeafSlot(rootWide * width + 0, in nodes[0]);
            validMask[rootWide] = 1;
        }
        else
        {
            queue.Enqueue((0, rootWide));
        }

        while (queue.Count > 0)
        {
            var (binIdx, wideId) = queue.Dequeue();
            Collapse(binIdx, nodes, children, width);

            var mask = 0;
            for (var c = 0; c < children.Count; c++)
            {
                var cbin = children[c];
                var slot = wideId * width + c;
                mask |= 1 << c;

                if (nodes[cbin].Count > 0)
                {
                    SetLeafSlot(slot, in nodes[cbin]);
                }
                else
                {
                    var cwid = AllocWide();
                    SetInternalSlot(slot, in nodes[cbin], cwid);
                    queue.Enqueue((cbin, cwid));
                }
            }

            validMask[wideId] = mask;
        }

        return new BvhSimd(
            v0, v1, v2, triIdx,
            minX.ToArray(), minY.ToArray(), minZ.ToArray(),
            maxX.ToArray(), maxY.ToArray(), maxZ.ToArray(),
            childIdx.ToArray(), childCount.ToArray(), validMask.ToArray(), validMask.Count, tier);

        void SetLeafSlot(int slot, in BvhNode n)
        {
            WriteAabb(slot, in n);
            childCount[slot] = n.Count;      // > 0
            childIdx[slot]   = n.LeftFirst;  // first index into _triIdx
        }

        void SetInternalSlot(int slot, in BvhNode n, int childWideId)
        {
            WriteAabb(slot, in n);
            childCount[slot] = 0;            // internal marker
            childIdx[slot]   = childWideId;  // child wide node index
        }

        void WriteAabb(int slot, in BvhNode n)
        {
            minX[slot] = n.Min.X; minY[slot] = n.Min.Y; minZ[slot] = n.Min.Z;
            maxX[slot] = n.Max.X; maxY[slot] = n.Max.Y; maxZ[slot] = n.Max.Z;
        }
    }

    /// <summary>
    ///     Collect up to <paramref name="width" /> binary-node indices to become one wide node's children: seed
    ///     with the binary node's two children, then repeatedly split the largest-surface-area INTERNAL member
    ///     into its own two children until we have <paramref name="width" /> members or every member is a leaf.
    /// </summary>
    private static void Collapse(int binIdx, BvhNode[] nodes, List<int> outChildren, int width)
    {
        outChildren.Clear();
        var l = nodes[binIdx].LeftFirst;
        outChildren.Add(l);
        outChildren.Add(l + 1);

        while (outChildren.Count < width)
        {
            var bestPos = -1;
            var bestSa  = -1f;
            for (var i = 0; i < outChildren.Count; i++)
            {
                ref readonly var n = ref nodes[outChildren[i]];
                if (n.Count != 0)
                    continue; // leaf — cannot expand
                var sa = SurfaceArea(n.Min, n.Max);
                if (sa > bestSa)
                {
                    bestSa  = sa;
                    bestPos = i;
                }
            }

            if (bestPos < 0)
                break; // all leaves

            var expand = outChildren[bestPos];
            outChildren.RemoveAt(bestPos);
            var el = nodes[expand].LeftFirst;
            outChildren.Add(el);
            outChildren.Add(el + 1);
        }
    }

    // ── Query ─────────────────────────────────────────────────────────────────────────────────────────────

    /// <summary>
    ///     True if any baked triangle intersects the open segment (<paramref name="from" />,
    ///     <paramref name="to" />) — line of sight blocked. Boolean-identical to <see cref="Bvh.Occluded" />.
    ///     Dispatches to the tier chosen at build. Thread-safe (read-only + stack-local scratch). Fail-open:
    ///     empty soup / coincident endpoints → false.
    /// </summary>
    public bool Occluded(Vector3 from, Vector3 to) => _tier switch
    {
        Tier.Avx512 => Occluded16(from, to),
        Tier.Avx2   => Occluded8(from, to),
        _           => Occluded4(from, to),
    };

    // Tier 3 — SSE / Vector128, 4 children per node.
    // SkipLocalsInit: the stackalloc int[256] scratch stack is otherwise zero-init on every ray (a memset per
    // query). Only [0, sp) is ever read (each entry written before read) and the heap-grow copies only that
    // range, so skipping the .locinit zeroing leaves the boolean answer bit-identical.
    [SkipLocalsInit]
    private bool Occluded4(Vector3 from, Vector3 to)
    {
        if (_v0.Length == 0) return false;
        var dir = to - from;
        if (dir.LengthSquared() < RayEps * RayEps) return false;

        var ox = Vector128.Create(from.X); var oy = Vector128.Create(from.Y); var oz = Vector128.Create(from.Z);
        var ix = Vector128.Create(1f / dir.X); var iy = Vector128.Create(1f / dir.Y); var iz = Vector128.Create(1f / dir.Z);
        var one = Vector128.Create(1f); var zero = Vector128<float>.Zero;

        Span<int> stack = stackalloc int[256];
        var sp = 0;
        stack[sp++] = 0;

        while (sp > 0)
        {
            var node = stack[--sp];
            var b    = node * 4;

            var tx1 = Vector128.Multiply(Vector128.Subtract(Vector128.LoadUnsafe(ref _minX[b]), ox), ix);
            var tx2 = Vector128.Multiply(Vector128.Subtract(Vector128.LoadUnsafe(ref _maxX[b]), ox), ix);
            var ty1 = Vector128.Multiply(Vector128.Subtract(Vector128.LoadUnsafe(ref _minY[b]), oy), iy);
            var ty2 = Vector128.Multiply(Vector128.Subtract(Vector128.LoadUnsafe(ref _maxY[b]), oy), iy);
            var tz1 = Vector128.Multiply(Vector128.Subtract(Vector128.LoadUnsafe(ref _minZ[b]), oz), iz);
            var tz2 = Vector128.Multiply(Vector128.Subtract(Vector128.LoadUnsafe(ref _maxZ[b]), oz), iz);

            var tmin = Vector128.Max(Vector128.Max(Vector128.Min(tx1, tx2), Vector128.Min(ty1, ty2)), Vector128.Min(tz1, tz2));
            var tmax = Vector128.Min(Vector128.Min(Vector128.Max(tx1, tx2), Vector128.Max(ty1, ty2)), Vector128.Max(tz1, tz2));
            var tminC = Vector128.Max(tmin, zero);

            // Miss iff provably outside [0,1] AND no NaN operand in the lane (allFinite). NaN lane → HIT (visit).
            var allFinite = Vector128.BitwiseAnd(
                Vector128.BitwiseAnd(
                    Vector128.BitwiseAnd(Vector128.Equals(tx1, tx1), Vector128.Equals(tx2, tx2)),
                    Vector128.BitwiseAnd(Vector128.Equals(ty1, ty1), Vector128.Equals(ty2, ty2))),
                Vector128.BitwiseAnd(Vector128.Equals(tz1, tz1), Vector128.Equals(tz2, tz2)));
            var miss     = Vector128.BitwiseOr(Vector128.LessThan(tmax, tminC), Vector128.GreaterThan(tmin, one));
            var realMiss = Vector128.BitwiseAnd(miss, allFinite);
            var hitBits  = (~(int) realMiss.ExtractMostSignificantBits()) & _validMask[node];

            if (ProcessHitMask(node, 4, hitBits, from, dir, ref stack, ref sp))
                return true;
        }

        return false;
    }

    // Tier 2 — AVX2 / Vector256, 8 children per node.
    [SkipLocalsInit] // see Occluded4: scratch stack read only in [0, sp) → skipping zero-init is answer-preserving.
    private bool Occluded8(Vector3 from, Vector3 to)
    {
        if (_v0.Length == 0) return false;
        var dir = to - from;
        if (dir.LengthSquared() < RayEps * RayEps) return false;

        var ox = Vector256.Create(from.X); var oy = Vector256.Create(from.Y); var oz = Vector256.Create(from.Z);
        var ix = Vector256.Create(1f / dir.X); var iy = Vector256.Create(1f / dir.Y); var iz = Vector256.Create(1f / dir.Z);
        var one = Vector256.Create(1f); var zero = Vector256<float>.Zero;

        Span<int> stack = stackalloc int[256];
        var sp = 0;
        stack[sp++] = 0;

        while (sp > 0)
        {
            var node = stack[--sp];
            var b    = node * 8;

            var tx1 = Vector256.Multiply(Vector256.Subtract(Vector256.LoadUnsafe(ref _minX[b]), ox), ix);
            var tx2 = Vector256.Multiply(Vector256.Subtract(Vector256.LoadUnsafe(ref _maxX[b]), ox), ix);
            var ty1 = Vector256.Multiply(Vector256.Subtract(Vector256.LoadUnsafe(ref _minY[b]), oy), iy);
            var ty2 = Vector256.Multiply(Vector256.Subtract(Vector256.LoadUnsafe(ref _maxY[b]), oy), iy);
            var tz1 = Vector256.Multiply(Vector256.Subtract(Vector256.LoadUnsafe(ref _minZ[b]), oz), iz);
            var tz2 = Vector256.Multiply(Vector256.Subtract(Vector256.LoadUnsafe(ref _maxZ[b]), oz), iz);

            var tmin = Vector256.Max(Vector256.Max(Vector256.Min(tx1, tx2), Vector256.Min(ty1, ty2)), Vector256.Min(tz1, tz2));
            var tmax = Vector256.Min(Vector256.Min(Vector256.Max(tx1, tx2), Vector256.Max(ty1, ty2)), Vector256.Max(tz1, tz2));
            var tminC = Vector256.Max(tmin, zero);

            var allFinite = Vector256.BitwiseAnd(
                Vector256.BitwiseAnd(
                    Vector256.BitwiseAnd(Vector256.Equals(tx1, tx1), Vector256.Equals(tx2, tx2)),
                    Vector256.BitwiseAnd(Vector256.Equals(ty1, ty1), Vector256.Equals(ty2, ty2))),
                Vector256.BitwiseAnd(Vector256.Equals(tz1, tz1), Vector256.Equals(tz2, tz2)));
            var miss     = Vector256.BitwiseOr(Vector256.LessThan(tmax, tminC), Vector256.GreaterThan(tmin, one));
            var realMiss = Vector256.BitwiseAnd(miss, allFinite);
            var hitBits  = (~(int) realMiss.ExtractMostSignificantBits()) & _validMask[node];

            if (ProcessHitMask(node, 8, hitBits, from, dir, ref stack, ref sp))
                return true;
        }

        return false;
    }

    // Tier 1 — AVX-512 / Vector512, 16 children per node (the primary path on the deployment box).
    [SkipLocalsInit] // see Occluded4: scratch stack read only in [0, sp) → skipping zero-init is answer-preserving.
    private bool Occluded16(Vector3 from, Vector3 to)
    {
        if (_v0.Length == 0) return false;
        var dir = to - from;
        if (dir.LengthSquared() < RayEps * RayEps) return false;

        var ox = Vector512.Create(from.X); var oy = Vector512.Create(from.Y); var oz = Vector512.Create(from.Z);
        var ix = Vector512.Create(1f / dir.X); var iy = Vector512.Create(1f / dir.Y); var iz = Vector512.Create(1f / dir.Z);
        var one = Vector512.Create(1f); var zero = Vector512<float>.Zero;

        Span<int> stack = stackalloc int[256];
        var sp = 0;
        stack[sp++] = 0;

        while (sp > 0)
        {
            var node = stack[--sp];
            var b    = node * 16;

            var tx1 = Vector512.Multiply(Vector512.Subtract(Vector512.LoadUnsafe(ref _minX[b]), ox), ix);
            var tx2 = Vector512.Multiply(Vector512.Subtract(Vector512.LoadUnsafe(ref _maxX[b]), ox), ix);
            var ty1 = Vector512.Multiply(Vector512.Subtract(Vector512.LoadUnsafe(ref _minY[b]), oy), iy);
            var ty2 = Vector512.Multiply(Vector512.Subtract(Vector512.LoadUnsafe(ref _maxY[b]), oy), iy);
            var tz1 = Vector512.Multiply(Vector512.Subtract(Vector512.LoadUnsafe(ref _minZ[b]), oz), iz);
            var tz2 = Vector512.Multiply(Vector512.Subtract(Vector512.LoadUnsafe(ref _maxZ[b]), oz), iz);

            var tmin = Vector512.Max(Vector512.Max(Vector512.Min(tx1, tx2), Vector512.Min(ty1, ty2)), Vector512.Min(tz1, tz2));
            var tmax = Vector512.Min(Vector512.Min(Vector512.Max(tx1, tx2), Vector512.Max(ty1, ty2)), Vector512.Max(tz1, tz2));
            var tminC = Vector512.Max(tmin, zero);

            var allFinite = Vector512.BitwiseAnd(
                Vector512.BitwiseAnd(
                    Vector512.BitwiseAnd(Vector512.Equals(tx1, tx1), Vector512.Equals(tx2, tx2)),
                    Vector512.BitwiseAnd(Vector512.Equals(ty1, ty1), Vector512.Equals(ty2, ty2))),
                Vector512.BitwiseAnd(Vector512.Equals(tz1, tz1), Vector512.Equals(tz2, tz2)));
            var miss     = Vector512.BitwiseOr(Vector512.LessThan(tmax, tminC), Vector512.GreaterThan(tmin, one));
            var realMiss = Vector512.BitwiseAnd(miss, allFinite);
            var hitBits  = (~(int) realMiss.ExtractMostSignificantBits()) & _validMask[node];

            if (ProcessHitMask(node, 16, hitBits, from, dir, ref stack, ref sp))
                return true;
        }

        return false;
    }

    /// <summary>
    ///     Shared leaf/child dispatch for one node's hit mask (tier-independent). For each set bit: a LEAF slot
    ///     runs the exact scalar <see cref="RayTriangle" /> over its triangles (returns true on first hit); an
    ///     INTERNAL slot is pushed onto <paramref name="stack" /> (grown onto the heap on overflow, mirroring
    ///     the scalar traversal — a dropped subtree could miss an occluder → a wallhack leak).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool ProcessHitMask(int node, int width, int hitBits, Vector3 from, Vector3 dir,
        ref Span<int> stack, ref int sp)
    {
        var b = node * width;
        while (hitBits != 0)
        {
            var c    = System.Numerics.BitOperations.TrailingZeroCount(hitBits);
            hitBits &= hitBits - 1;

            var slot = b + c;
            var cnt  = _childCount[slot];
            if (cnt > 0)
            {
                var first = _childIdx[slot];
                var end   = first + cnt;
                for (var k = first; k < end; k++)
                {
                    if (RayTriangle(from, dir, _triIdx[k]))
                        return true;
                }
            }
            else
            {
                if (sp + 1 > stack.Length)
                {
                    var bigger = new int[stack.Length * 2];
                    stack.Slice(0, sp).CopyTo(bigger);
                    stack = bigger;
                }

                stack[sp++] = _childIdx[slot];
            }
        }

        return false;
    }

    /// <summary>
    ///     Double-sided Möller–Trumbore — <b>character-for-character the scalar <see cref="Bvh.RayTriangle" />
    ///     arithmetic</b> (same <see cref="Vector3" /> ops, same epsilons, no FMA). Keeping the leaf test in
    ///     scalar float math is what makes every per-triangle decision bit-identical to the scalar reference, so
    ///     the boolean result of a whole query cannot diverge — on any tier. True if hit at t ∈ (eps, 1].
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool RayTriangle(Vector3 orig, Vector3 dir, int tri)
    {
        var a     = _v0[tri];
        var edge1 = _v1[tri] - a;
        var edge2 = _v2[tri] - a;

        var h   = Vector3.Cross(dir, edge2);
        var det = Vector3.Dot(edge1, h);
        if (det > -1e-9f && det < 1e-9f)
            return false;

        var invDet = 1f / det;
        var s      = orig - a;
        var u      = invDet * Vector3.Dot(s, h);
        if (u < 0f || u > 1f)
            return false;

        var q = Vector3.Cross(s, edge1);
        var v = invDet * Vector3.Dot(dir, q);
        if (v < 0f || u + v > 1f)
            return false;

        var t = invDet * Vector3.Dot(edge2, q);
        return t > RayEps && t <= 1f;
    }

    /// <summary>Half the surface area of an AABB (SAH proxy for the collapse heuristic; monotone, ½ elided).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float SurfaceArea(Vector3 min, Vector3 max)
    {
        var e = max - min;
        if (e.X < 0f || e.Y < 0f || e.Z < 0f)
            return 0f;
        return e.X * e.Y + e.Y * e.Z + e.Z * e.X;
    }
}
