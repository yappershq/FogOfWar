using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace FogOfWar.Engine;

/// <summary>A single BVH node. 32 bytes, blittable (memcpy-serialisable to the bake cache).</summary>
[StructLayout(LayoutKind.Sequential)]
internal struct BvhNode
{
    public Vector3 Min;
    public int     LeftFirst; // internal (Count==0): left child index, right child = LeftFirst+1.
                              // leaf     (Count>0):  first index into the triangle-index permutation.
    public Vector3 Max;
    public int     Count;     // 0 => internal node; >0 => leaf triangle count.
}

/// <summary>
///     A binary bounding-volume hierarchy over a static world-space triangle soup, built with binned SAH
///     (surface-area heuristic) and queried with a stack-based segment/occlusion traversal + double-sided
///     Möller–Trumbore ray-triangle test.
///
///     <para><b>Immutable after construction → thread-safe.</b> <see cref="Occluded" /> only reads the node /
///     triangle arrays and uses stack-local scratch, so it may be called concurrently from any thread
///     (including a worker thread off the game loop).</para>
/// </summary>
internal sealed class Bvh
{
    // Geometry (struct-of-arrays, original order).
    private readonly Vector3[] _v0;
    private readonly Vector3[] _v1;
    private readonly Vector3[] _v2;

    private readonly BvhNode[] _nodes;    // node pool (index 0 == root)
    private readonly int[]     _triIdx;   // triangle-index permutation; leaves reference contiguous ranges
    private readonly int       _nodeCount;

    private const int Bins        = 12;   // SAH bins per axis
    private const int MaxLeafTris = 4;    // stop subdividing at/under this many triangles
    private const float RayEps    = 1e-4f;

    internal Bvh(Vector3[] v0, Vector3[] v1, Vector3[] v2, BvhNode[] nodes, int[] triIdx, int nodeCount)
    {
        _v0        = v0;
        _v1        = v1;
        _v2        = v2;
        _nodes     = nodes;
        _triIdx    = triIdx;
        _nodeCount = nodeCount;
    }

    public int NodeCount     => _nodeCount;
    public int TriangleCount => _v0.Length;

    /// <summary>
    ///     O(nodes) structural validation for a BVH deserialised from an UNTRUSTED bake cache. A corrupt internal
    ///     node whose child index points back at an ancestor would make <see cref="Occluded" />'s stack traversal
    ///     cycle forever (worker pegs a core; <c>Join</c> times out; ALC leak on hot-reload). This guarantees the
    ///     node graph is a finite acyclic tree with in-range indices; a freshly <see cref="Build" />-t BVH always
    ///     passes, so the caller only needs it on the cache path.
    ///
    ///     <para>Invariants: every INTERNAL node (<c>Count == 0</c>) has children at <c>LeftFirst</c> and
    ///     <c>LeftFirst + 1</c>, both strictly greater than the node's own index (child-index-greater-than-parent
    ///     ⇒ acyclic) and in range; every LEAF (<c>Count &gt; 0</c>) references a contiguous, in-range slice of
    ///     the triangle-index permutation, and every referenced permutation entry is a valid triangle index.</para>
    /// </summary>
    internal bool Validate()
    {
        var triCount = _v0.Length;

        if (_nodeCount < 1 || _nodeCount > _nodes.Length)
            return false;

        // Degenerate empty BVH (no occluder triangles): a single empty leaf. Build() emits Count==0 here, which
        // would otherwise trip the internal-node rule below — accept it explicitly.
        if (_nodeCount == 1 && triCount == 0)
            return _triIdx.Length == 0;

        for (var i = 0; i < _nodeCount; i++)
        {
            ref readonly var node = ref _nodes[i];

            // Negative Count is nonsense (corrupt/tampered cache): Occluded would treat it as an internal
            // node and chase an unconstrained LeftFirst → possible traversal cycle → worker hang. Reject.
            if (node.Count < 0)
                return false;

            if (node.Count == 0)
            {
                // Internal: both children must have a HIGHER index than this node (acyclicity) and be in range.
                if (node.LeftFirst <= i || node.LeftFirst + 1 >= _nodeCount)
                    return false;
            }
            else
            {
                // Leaf: [LeftFirst, LeftFirst+Count) must lie inside the triangle-index permutation...
                if (node.LeftFirst < 0 || (long) node.LeftFirst + node.Count > _triIdx.Length)
                    return false;

                // ...and every referenced permutation entry must be a valid triangle index.
                var end = node.LeftFirst + node.Count;
                for (var k = node.LeftFirst; k < end; k++)
                {
                    if ((uint) _triIdx[k] >= (uint) triCount)
                        return false;
                }
            }
        }

        return true;
    }

    // Serialisation access (BakeCache, same assembly).
    internal Vector3[] V0     => _v0;
    internal Vector3[] V1     => _v1;
    internal Vector3[] V2     => _v2;
    internal BvhNode[] Nodes  => _nodes;
    internal int[]     TriIdx => _triIdx;

    // ── Build ────────────────────────────────────────────────────────────────────────────────────────────

    public static Bvh Build(Vector3[] v0, Vector3[] v1, Vector3[] v2)
    {
        var n = v0.Length;

        // Degenerate empty case: a single empty leaf that never intersects anything.
        if (n == 0)
        {
            var empty = new BvhNode
            {
                Min = new Vector3(float.PositiveInfinity),
                Max = new Vector3(float.NegativeInfinity),
                LeftFirst = 0,
                Count = 0,
            };
            return new Bvh(v0, v1, v2, new[] { empty }, Array.Empty<int>(), 1);
        }

        var triIdx    = new int[n];
        var triMin     = new Vector3[n];
        var triMax     = new Vector3[n];
        var centroid   = new Vector3[n];
        for (var i = 0; i < n; i++)
        {
            triIdx[i] = i;
            var a = v0[i]; var b = v1[i]; var c = v2[i];
            var lo = Vector3.Min(Vector3.Min(a, b), c);
            var hi = Vector3.Max(Vector3.Max(a, b), c);
            triMin[i]   = lo;
            triMax[i]   = hi;
            centroid[i] = (lo + hi) * 0.5f;
        }

        // Node pool upper bound for a binary BVH over n primitives is 2n-1.
        var nodes = new BvhNode[Math.Max(1, 2 * n - 1)];
        var nodesUsed = 1;

        nodes[0].LeftFirst = 0;
        nodes[0].Count     = n;
        UpdateBounds(ref nodes[0], triIdx, triMin, triMax);

        // Explicit stack (avoids deep recursion / stack overflow on ~1.5M-triangle maps).
        var stack = new int[64];
        var sp = 0;
        stack[sp++] = 0;

        while (sp > 0)
        {
            var nodeIdx = stack[--sp];
            ref var node = ref nodes[nodeIdx];

            if (node.Count <= MaxLeafTris)
                continue;

            if (!FindBestSplit(in node, triIdx, triMin, triMax, centroid, out var axis, out var splitPos,
                    out var splitCost))
                continue;

            // Compare split cost against the cost of keeping this node a leaf (SAH: N * area).
            var leafCost = node.Count * SurfaceArea(node.Min, node.Max);
            if (splitCost >= leafCost)
                continue;

            // Partition triangle indices in-place by centroid on the chosen axis.
            var i = node.LeftFirst;
            var j = i + node.Count - 1;
            while (i <= j)
            {
                if (Axis(centroid[triIdx[i]], axis) < splitPos)
                {
                    i++;
                }
                else
                {
                    (triIdx[i], triIdx[j]) = (triIdx[j], triIdx[i]);
                    j--;
                }
            }

            var leftCount = i - node.LeftFirst;
            if (leftCount == 0 || leftCount == node.Count)
                continue; // degenerate split → keep as leaf

            var leftIdx  = nodesUsed++;
            var rightIdx = nodesUsed++;

            nodes[leftIdx].LeftFirst  = node.LeftFirst;
            nodes[leftIdx].Count      = leftCount;
            nodes[rightIdx].LeftFirst = i;
            nodes[rightIdx].Count     = node.Count - leftCount;

            node.LeftFirst = leftIdx;
            node.Count     = 0; // now internal

            UpdateBounds(ref nodes[leftIdx],  triIdx, triMin, triMax);
            UpdateBounds(ref nodes[rightIdx], triIdx, triMin, triMax);

            if (sp + 2 > stack.Length)
                Array.Resize(ref stack, stack.Length * 2);
            stack[sp++] = leftIdx;
            stack[sp++] = rightIdx;
        }

        return new Bvh(v0, v1, v2, nodes, triIdx, nodesUsed);
    }

    private static void UpdateBounds(ref BvhNode node, int[] triIdx, Vector3[] triMin, Vector3[] triMax)
    {
        var lo = new Vector3(float.PositiveInfinity);
        var hi = new Vector3(float.NegativeInfinity);
        var end = node.LeftFirst + node.Count;
        for (var i = node.LeftFirst; i < end; i++)
        {
            var t = triIdx[i];
            lo = Vector3.Min(lo, triMin[t]);
            hi = Vector3.Max(hi, triMax[t]);
        }

        node.Min = lo;
        node.Max = hi;
    }

    /// <summary>Binned-SAH best split over all three axes. False when no useful split exists (make a leaf).</summary>
    private static bool FindBestSplit(
        in BvhNode node, int[] triIdx, Vector3[] triMin, Vector3[] triMax, Vector3[] centroid,
        out int bestAxis, out float bestPos, out float bestCost)
    {
        bestAxis = -1;
        bestPos  = 0f;
        bestCost = float.PositiveInfinity;

        var start = node.LeftFirst;
        var end   = node.LeftFirst + node.Count;

        // Scratch hoisted out of the axis loop (fixed small size; avoids stackalloc-in-loop).
        Span<int>     binCount  = stackalloc int[Bins];
        Span<Vector3> binMin    = stackalloc Vector3[Bins];
        Span<Vector3> binMax    = stackalloc Vector3[Bins];
        Span<float>   leftArea  = stackalloc float[Bins - 1];
        Span<float>   rightArea = stackalloc float[Bins - 1];
        Span<int>     leftCnt   = stackalloc int[Bins - 1];
        Span<int>     rightCnt  = stackalloc int[Bins - 1];

        for (var axis = 0; axis < 3; axis++)
        {
            // Centroid bounds on this axis.
            var boundsMin = float.PositiveInfinity;
            var boundsMax = float.NegativeInfinity;
            for (var i = start; i < end; i++)
            {
                var c = Axis(centroid[triIdx[i]], axis);
                if (c < boundsMin) boundsMin = c;
                if (c > boundsMax) boundsMax = c;
            }

            if (boundsMax - boundsMin < 1e-6f)
                continue; // all centroids coincide on this axis

            for (var b = 0; b < Bins; b++)
            {
                binCount[b] = 0;
                binMin[b]   = new Vector3(float.PositiveInfinity);
                binMax[b]   = new Vector3(float.NegativeInfinity);
            }

            var scale = Bins / (boundsMax - boundsMin);
            for (var i = start; i < end; i++)
            {
                var t = triIdx[i];
                var b = (int)((Axis(centroid[t], axis) - boundsMin) * scale);
                if (b < 0) b = 0;
                if (b >= Bins) b = Bins - 1;
                binCount[b]++;
                binMin[b] = Vector3.Min(binMin[b], triMin[t]);
                binMax[b] = Vector3.Max(binMax[b], triMax[t]);
            }

            // Sweep from both sides to accumulate area * count for each of the Bins-1 split planes.
            var lMin = new Vector3(float.PositiveInfinity);
            var lMax = new Vector3(float.NegativeInfinity);
            var lSum = 0;
            var rMin = new Vector3(float.PositiveInfinity);
            var rMax = new Vector3(float.NegativeInfinity);
            var rSum = 0;

            for (var b = 0; b < Bins - 1; b++)
            {
                lSum += binCount[b];
                leftCnt[b] = lSum;
                lMin = Vector3.Min(lMin, binMin[b]);
                lMax = Vector3.Max(lMax, binMax[b]);
                leftArea[b] = lSum == 0 ? 0f : SurfaceArea(lMin, lMax);

                var rb = Bins - 1 - b;
                rSum += binCount[rb];
                rightCnt[Bins - 2 - b] = rSum;
                rMin = Vector3.Min(rMin, binMin[rb]);
                rMax = Vector3.Max(rMax, binMax[rb]);
                rightArea[Bins - 2 - b] = rSum == 0 ? 0f : SurfaceArea(rMin, rMax);
            }

            var invScale = (boundsMax - boundsMin) / Bins;
            for (var b = 0; b < Bins - 1; b++)
            {
                var cost = leftCnt[b] * leftArea[b] + rightCnt[b] * rightArea[b];
                if (cost > 0f && cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = axis;
                    bestPos  = boundsMin + (b + 1) * invScale;
                }
            }
        }

        return bestAxis != -1;
    }

    // ── Query ────────────────────────────────────────────────────────────────────────────────────────────

    /// <summary>
    ///     True if any baked triangle intersects the open segment (<paramref name="from" />,
    ///     <paramref name="to" />) — i.e. the line of sight is blocked. Thread-safe (read-only + stack scratch).
    ///
    ///     <para><see cref="SkipLocalsInitAttribute" />: the <c>stackalloc int[128]</c> scratch stack below is
    ///     zero-initialised by the runtime on EVERY ray otherwise (a memset per query, ~1GB/s at 64 players).
    ///     The traversal only ever reads slots in <c>[0, sp)</c> (each entry is WRITTEN before it is read), and the
    ///     heap-grow path copies only that used range, so skipping the .locinit zeroing cannot change any result.</para>
    /// </summary>
    [SkipLocalsInit]
    public bool Occluded(Vector3 from, Vector3 to)
    {
        // Empty occluder set: a 0-triangle soup builds a single node with Count==0 (the internal-node marker),
        // so without this guard the traversal below would take the internal path and read _nodes[1] (OOB — the
        // pool has only _nodes[0]) on EVERY ray, throwing every worker pass → FOW permanently inert. No triangles
        // ⇒ nothing can ever block the line of sight, so short-circuit VISIBLE.
        if (_v0.Length == 0)
            return false;

        var dir = to - from;
        var lenSq = dir.LengthSquared();
        if (lenSq < RayEps * RayEps)
            return false; // coincident endpoints → nothing between them

        // Un-normalised direction: the segment is parametrised t ∈ [0,1]. invDir components may be ±Inf when a
        // direction component is 0; the slab test's NaN-discarding min/max handles that correctly.
        var invDir = new Vector3(1f / dir.X, 1f / dir.Y, 1f / dir.Z);

        // Fast path: a stackalloc scratch stack (no heap alloc). A pathologically deep tree (degenerate SAH
        // splits) can exceed the initial cap; on overflow we GROW onto the heap (mirroring Build) rather than
        // dropping a subtree — a dropped subtree could miss an occluder → a wallhack LEAK.
        Span<int> stack = stackalloc int[128];
        var sp = 0;
        stack[sp++] = 0;

        while (sp > 0)
        {
            ref readonly var node = ref _nodes[stack[--sp]];

            if (node.Count > 0)
            {
                var first = node.LeftFirst;
                var end   = first + node.Count;
                for (var k = first; k < end; k++)
                {
                    if (RayTriangle(from, dir, _triIdx[k]))
                        return true;
                }

                continue;
            }

            var left  = node.LeftFirst;
            var right = left + 1;

            var hitL = SlabHit(in _nodes[left],  from, invDir);
            var hitR = SlabHit(in _nodes[right], from, invDir);

            // Ensure room for BOTH pushes before either write (guard both, not just the right push). Grow onto
            // the heap on overflow; the Span is simply retargeted at the larger array and prior entries copied.
            if (sp + 2 > stack.Length)
            {
                var bigger = new int[stack.Length * 2];
                stack.Slice(0, sp).CopyTo(bigger);
                stack = bigger;
            }

            if (hitL) stack[sp++] = left;
            if (hitR) stack[sp++] = right;
        }

        return false;
    }

    /// <summary>
    ///     Ray/AABB slab test against the segment interval [0,1]. Handles ±Inf invDir components (an
    ///     axis-aligned ray). <b>Conservative NaN handling (no wallhack leak):</b> when a direction component
    ///     is 0 (invDir = ±Inf) and the ray grazes a slab plane exactly (<c>from[axis] == node bound</c>), one
    ///     <c>t</c> is <c>0 * ±Inf = NaN</c>. A NaN that survives into <c>tMin</c>/<c>tMax</c> must resolve to
    ///     HIT (visit the node) — NOT to a spurious skip, which would miss a real occluder and leak an enemy to
    ///     a wallhack. So the per-axis extents use <see cref="PMin" /> / <see cref="PMax" /> that PROPAGATE NaN
    ///     (an axis with a NaN operand makes both its min and max NaN together), and the result is the NEGATION
    ///     of the miss condition: ordered comparisons are false on NaN, so a NaN interval yields
    ///     <c>!(false || false) == true</c> (visit). A node whose AABB bounds a truly-hit triangle is therefore
    ///     never pruned; the exact <see cref="RayTriangle" /> at the leaf makes the final call, so this can only
    ///     ever add a few extra node visits, never change a boolean answer for a non-degenerate ray.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SlabHit(in BvhNode node, Vector3 from, Vector3 invDir)
    {
        var tx1 = (node.Min.X - from.X) * invDir.X;
        var tx2 = (node.Max.X - from.X) * invDir.X;
        var ty1 = (node.Min.Y - from.Y) * invDir.Y;
        var ty2 = (node.Max.Y - from.Y) * invDir.Y;
        var tz1 = (node.Min.Z - from.Z) * invDir.Z;
        var tz2 = (node.Max.Z - from.Z) * invDir.Z;

        var tMin = PMax(PMax(PMin(tx1, tx2), PMin(ty1, ty2)), PMin(tz1, tz2));
        var tMax = PMin(PMin(PMax(tx1, tx2), PMax(ty1, ty2)), PMax(tz1, tz2));

        // Clamp tMin up to 0 (segment starts at t=0). If tMin is NaN this yields 0, but then tMax is NaN too
        // (both come from the same axis' operands), so the miss-negation below still resolves the lane to HIT.
        var tMinClamped = tMin > 0f ? tMin : 0f;

        // Miss iff the interval is provably outside [0,1]. NaN → neither comparison true → NOT a miss → visit.
        return !(tMax < tMinClamped || tMin > 1f);
    }

    /// <summary>NaN-propagating min: returns NaN if either operand is NaN, else the smaller.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float PMin(float a, float b)
        => float.IsNaN(a) ? a : float.IsNaN(b) ? b : a < b ? a : b;

    /// <summary>NaN-propagating max: returns NaN if either operand is NaN, else the larger.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float PMax(float a, float b)
        => float.IsNaN(a) ? a : float.IsNaN(b) ? b : a > b ? a : b;

    /// <summary>Double-sided Möller–Trumbore. True if the triangle is hit at t ∈ (eps, 1].</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool RayTriangle(Vector3 orig, Vector3 dir, int tri)
    {
        var a = _v0[tri];
        var edge1 = _v1[tri] - a;
        var edge2 = _v2[tri] - a;

        var h = Vector3.Cross(dir, edge2);
        var det = Vector3.Dot(edge1, h);
        if (det > -1e-9f && det < 1e-9f)
            return false; // ray parallel to triangle

        var invDet = 1f / det;
        var s = orig - a;
        var u = invDet * Vector3.Dot(s, h);
        if (u < 0f || u > 1f)
            return false;

        var q = Vector3.Cross(s, edge1);
        var v = invDet * Vector3.Dot(dir, q);
        if (v < 0f || u + v > 1f)
            return false;

        var t = invDet * Vector3.Dot(edge2, q);
        return t > RayEps && t <= 1f;
    }

    // ── math helpers ─────────────────────────────────────────────────────────────────────────────────────

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float Axis(Vector3 v, int axis) => axis == 0 ? v.X : axis == 1 ? v.Y : v.Z;

    /// <summary>Half the surface area of an AABB (SAH cost proxy; monotone, so the ½ is elided).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float SurfaceArea(Vector3 min, Vector3 max)
    {
        var e = max - min;
        if (e.X < 0f || e.Y < 0f || e.Z < 0f)
            return 0f; // empty
        return e.X * e.Y + e.Y * e.Z + e.Z * e.X;
    }
}
