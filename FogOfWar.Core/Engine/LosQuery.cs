using System.Numerics;

namespace FogOfWar.Engine;

/// <summary>
///     Public line-of-sight oracle for the Fog-of-War module: wraps a <see cref="Bvh" /> over a map's static
///     world collision (opacity-filtered) and answers <see cref="IsBlocked" /> — ray-vs-static-geometry
///     line-of-sight. Built once per map (from geometry or the bake cache) and <b>immutable thereafter →
///     thread-safe</b>: <see cref="IsBlocked" /> may be called concurrently from a worker thread off the game
///     loop.
///
///     <para><see cref="Crc32" /> + <see cref="Size" /> are the packed <c>world_physics</c> VPK CRC and length,
///     used by <see cref="BakeCache" /> to key the serialised BVH so a rebuild is skipped when the map's
///     physics data is unchanged.</para>
/// </summary>
internal sealed class LosQuery
{
    private readonly Bvh       _bvh;   // scalar reference engine (always present).
    private readonly BvhSimd?  _simd;  // optional tiered SIMD engine (AVX-512/AVX2/SSE) — null ⇒ scalar fallback.

    public uint Crc32 { get; }
    public long Size  { get; }

    public int TriangleCount => _bvh.TriangleCount;
    public int NodeCount     => _bvh.NodeCount;

    /// <summary>True when the fast SSE engine is built and in use (else <see cref="IsBlocked" /> is scalar).</summary>
    public bool SimdEnabled => _simd is not null;

    internal Bvh      Bvh  => _bvh;
    internal BvhSimd? Simd => _simd;

    internal LosQuery(Bvh bvh, uint crc32, long size, BvhSimd? simd = null)
    {
        _bvh   = bvh;
        _simd  = simd;
        Crc32  = crc32;
        Size   = size;
    }

    /// <summary>Build a query from loaded, filtered geometry (scalar engine only — the correctness reference).</summary>
    public static LosQuery Build(CollisionGeometry geometry)
        => new(Bvh.Build(geometry.V0, geometry.V1, geometry.V2), geometry.Crc32, geometry.Size);

    /// <summary>
    ///     Build a query with the fast tiered SIMD (<see cref="BvhSimd" />) engine, transparently falling back
    ///     to the scalar engine when no SIMD is available. The SIMD engine widens the SAME scalar BVH and shares
    ///     its triangle arrays, so it answers boolean-identically on every tier — see <see cref="BvhSimd" />.
    ///
    ///     <para><paramref name="pinTier" /> forces a specific tier (falling back to the widest the CPU supports
    ///     if that tier is unavailable). Left null it auto-selects the widest (AVX-512 → AVX2 → SSE4.1). NOTE:
    ///     benchmarks on typical CS2 maps (~100k occluder tris → a shallow wide tree) show AVX2 is marginally
    ///     the fastest tier; AVX-512's 16-wide traversal only pulls ahead on much larger maps. Pin
    ///     <see cref="BvhSimd.Tier.Avx2" /> here if profiling favours it on the deployment's map mix.</para>
    /// </summary>
    public static LosQuery BuildWithSimd(CollisionGeometry geometry, BvhSimd.Tier? pinTier = null)
    {
        var bvh = Bvh.Build(geometry.V0, geometry.V1, geometry.V2);
        BvhSimd? simd = null;
        if (BvhSimd.IsSupported)
        {
            var tier = pinTier is { } p && BvhSimd.TierSupported(p) ? p : BvhSimd.BestTier();
            simd = BvhSimd.Build(bvh, tier);
        }

        return new LosQuery(bvh, geometry.Crc32, geometry.Size, simd);
    }

    /// <summary>Attach a SIMD engine to a query already built (e.g. loaded from the bake cache).</summary>
    internal LosQuery WithSimd(BvhSimd.Tier? pinTier = null)
    {
        if (_simd is not null || !BvhSimd.IsSupported)
            return this;

        var tier = pinTier is { } p && BvhSimd.TierSupported(p) ? p : BvhSimd.BestTier();
        return new LosQuery(_bvh, Crc32, Size, BvhSimd.Build(_bvh, tier));
    }

    /// <summary>
    ///     True if static world geometry blocks the line of sight between <paramref name="from" /> and
    ///     <paramref name="to" /> (any baked triangle intersects the segment). Coordinates are Source2 Z-up
    ///     world space — the same frame as CS2 entity origins / eye positions. Uses the SIMD engine when built,
    ///     otherwise the scalar engine; both return the identical boolean.
    /// </summary>
    public bool IsBlocked(Vector3 from, Vector3 to)
        => _simd is not null ? _simd.Occluded(from, to) : _bvh.Occluded(from, to);

    /// <summary>The scalar-engine answer, always — the correctness reference the SIMD path is validated against.</summary>
    public bool IsBlockedScalar(Vector3 from, Vector3 to) => _bvh.Occluded(from, to);
}
