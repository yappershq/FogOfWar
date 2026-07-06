using System.Collections.Generic;
using System.Numerics;

namespace FogOfWar.Engine;

/// <summary>
///     Immutable world-space triangle soup produced by <see cref="MapCollisionLoader" /> from a map's
///     <c>world_physics</c> resource, after the <see cref="OpacityFilter" /> has removed see-through surfaces.
///     Stored struct-of-arrays (one <see cref="Vector3" /> array per triangle corner) for cache-friendly BVH
///     traversal. Coordinates are Source2 Z-up world space — the same frame as CS2 entity origins, so no
///     conversion is applied.
///
///     <para><see cref="Crc32" /> + <see cref="Size" /> come from the packed <c>world_physics</c>
///     <c>PackageEntry</c> (VPK CRC + uncompressed length) and are the bake-cache key material.</para>
/// </summary>
internal sealed class CollisionGeometry
{
    public Vector3[] V0 { get; }
    public Vector3[] V1 { get; }
    public Vector3[] V2 { get; }

    /// <summary>VPK CRC32 of the packed <c>world_physics</c> resource — bake-cache key.</summary>
    public uint Crc32 { get; }

    /// <summary>Uncompressed byte length of the packed <c>world_physics</c> resource — bake-cache key.</summary>
    public long Size { get; }

    public LoadStats Stats { get; }

    public int TriangleCount => V0.Length;

    public CollisionGeometry(Vector3[] v0, Vector3[] v1, Vector3[] v2, uint crc32, long size, LoadStats stats)
    {
        V0    = v0;
        V1    = v1;
        V2    = v2;
        Crc32 = crc32;
        Size  = size;
        Stats = stats;
    }
}

/// <summary>Diagnostics from a load: kept/excluded triangle counts and per-surface-property histograms.</summary>
internal sealed class LoadStats
{
    public long MeshTriangles;          // raw triangle-mesh triangles seen (pre-filter)
    public long HullTriangles;          // raw convex-hull triangles seen (pre-filter)
    public long SphereShapes;           // procedural primitives (never baked)
    public long CapsuleShapes;
    public long IncludedTriangles;      // kept as occluders
    public long ExcludedByOpacityLayer; // dropped by the PRIMARY generic interaction-layer test (passbullets/window)
    public long ExcludedSeeThrough;     // dropped by the SECONDARY see-through NAME + geometry test (thin, or glass)
    public long IncludedThickDespiteName; // KEPT solid despite a see-through family name — shape was geometrically THICK
    public long ExcludedClip;           // dropped by the (optional) invisible-clip filter

    /// <summary>Kept-triangle count by surface-property name.</summary>
    public readonly Dictionary<string, long> IncludedByProp = new();

    /// <summary>Excluded-triangle count by surface-property name (see-through materials that were also thin/glass).</summary>
    public readonly Dictionary<string, long> ExcludedByProp = new();

    /// <summary>
    ///     Kept-triangle count by surface-property name for triangles whose surfaceprop name matched a see-through
    ///     family but were BAKED SOLID anyway because their collision shape was geometrically thick (e.g. de_nuke
    ///     <c>metalvent</c> crawl-ducts). The thickness gate's leak-guard, surfaced for diagnosis.
    /// </summary>
    public readonly Dictionary<string, long> IncludedThickByProp = new();

    /// <summary>
    ///     Opacity-layer-excluded triangle count by collision-attribute tag-set string — i.e. which of the
    ///     game's own interaction layers (e.g. <c>passbullets</c>, <c>window</c>) drove each generic exclusion.
    /// </summary>
    public readonly Dictionary<string, long> ExcludedByOpacityTag = new();

    /// <summary>Triangle count by collision-attribute tag-set string (for clip analysis / reporting).</summary>
    public readonly Dictionary<string, long> ByCollisionTags = new();

    public Vector3 WorldMin;
    public Vector3 WorldMax;

    /// <summary>Full VPK path of the <c>world_physics</c> resource that was actually baked (for logging).</summary>
    public string ChosenPath = string.Empty;

    /// <summary>
    ///     Number of cache-key candidates competing in the outer package (direct <c>world_physics</c> resource
    ///     entries + nested map <c>.vpk</c> entries). &gt; 1 means the map was ambiguous (multi-map addon /
    ///     stub+real) and the selection could have picked the wrong geometry — surfaced as a bake WARN.
    /// </summary>
    public int KeyCandidateCount;

    /// <summary>
    ///     Number of <c>world_physics</c> resource entries in the package that was actually parsed (outer for a
    ///     direct hit, the chosen nested vpk otherwise). &gt; 1 means multiple maps were packed together.
    /// </summary>
    public int WorldPhysicsCandidateCount;
}
