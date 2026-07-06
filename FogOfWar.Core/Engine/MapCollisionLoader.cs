using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using SteamDatabase.ValvePak;
using ValveKeyValue;
using ValveResourceFormat;
using ValveResourceFormat.ResourceTypes;
using ValveResourceFormat.Serialization.KeyValues;
using ValveResourceFormat.Utils;

namespace FogOfWar.Engine;

/// <summary>
///     Loads a CS2 map's static collision geometry (<c>world_physics</c> = world brushes + baked static props)
///     from a map <c>.vpk</c> via ValveResourceFormat and returns it as a world-space triangle soup, after the
///     <see cref="OpacityFilter" /> has removed see-through surfaces.
///
///     <para>Pipeline (mirrors the proven fow-vrf-spike, which follows VRF's
///     <c>GltfModelExporter.LoadPhysicsMeshes</c> / <c>MapExtract.LoadWorldPhysics</c>):
///     <c>Package.Read</c> → find <c>world_physics.{vphys_c|vmdl_c}</c> (recursing into a nested
///     <c>maps/&lt;name&gt;.vpk</c> if the map is packed) → <c>Resource.Read</c> → <c>PhysAggregateData</c> →
///     per <c>Part</c>: transform by <c>BindPose</c> (identity for world_physics) →
///     <c>Shape.Meshes</c> (<c>GetVertices</c>/<c>GetTriangles</c>) + <c>Shape.Hulls</c>
///     (<c>GetVertexPositions</c>/<c>GetFaces</c>, fan-triangulated). Spheres/capsules are procedural and not
///     baked.</para>
///
///     <para><b>Opacity decision (generic, zero per-map hardcoding):</b> the PRIMARY signal is the game's own
///     per-shape <b>interaction layers</b> (VRF exposes them as the collision attribute's
///     <c>m_InteractAsStrings</c> tag set — see <c>SafeTagStrings</c>). A shape whose layers mark it
///     <i>non-vision-blocking</i> — CS2's authored see-through layers <c>passbullets</c>
///     (<c>InteractionLayers.PassBullets</c>) / <c>window</c> (<c>InteractionLayers.Window</c>) — is dropped as
///     see-through, generically across every map. A layer that <i>definitively</i> blocks vision
///     (<c>blocklos</c> = <c>InteractionLayers.BlockLos</c>, <c>opaque</c> = <c>InteractionLayers.CStrikeOpaque</c>)
///     force-bakes the shape and overrides the fallbacks below (leak-safe).</para>
///
///     <para><b>Why layers, not a numeric mask:</b> the runtime <c>InteractsAs</c> bitfield (with the
///     <c>CStrikeOpaque = 1&lt;&lt;36</c> bit) is <i>engine-derived at map load and is NOT stored in the compiled
///     <c>world_physics</c></i>. Empirically (de_mirage) the vphys carries only the authoring layer <b>strings</b>
///     plus a hash array — never a numeric <c>m_nInteractsAs</c> — and solid walls (concrete/plaster/brick) have
///     an EMPTY layer set (opacity is the default), while only see-through elements are tagged. So the opacity
///     test is the <i>inverse</i> of a positive-opaque bit test: opaque unless a see-through layer says otherwise.</para>
///
///     <para><b>Secondary / fallback:</b> the per-triangle surface-property name (<c>Mesh.Materials</c> /
///     <c>SurfacePropertyIndex</c> → <c>StringToken.GetKnownString</c>) is matched against the
///     <see cref="OpacityFilter" />'s known see-through set. This catches see-through geometry the compiled
///     physics leaves on the DEFAULT collision attribute with NO interaction tag (e.g. de_mirage
///     <c>metalvent</c>/<c>glass</c>), which the layer test alone cannot see. Invisible clip shapes can also be
///     dropped — OFF by default (conservative INCLUDE), clip count always reported.</para>
/// </summary>
internal static class MapCollisionLoader
{
    /// <summary>
    ///     Maximum <b>minimum-AABB-dimension</b> (Source units) at which a collision shape whose surfaceprop name
    ///     matches a see-through <see cref="OpacityFilter" /> family is treated as genuinely see-through and dropped
    ///     from the occluder bake. The surfaceprop name alone is ambiguous across official maps — <c>metalvent</c>
    ///     tags a THIN see-through lattice on de_mirage (min-dim 1–11u) but SOLID crawl-duct meshes on de_nuke
    ///     (min-dim ~346u+, players crawl inside). Gating exclusion on per-shape thinness keeps the nuke ducts solid
    ///     (no wallhack leak) while still dropping the mirage fences. Empirically the two populations are far apart:
    ///     real see-through fences/grates/vents/slats are ≤ ~11u thick, while the thinnest SOLID family-named look-
    ///     alike (de_mirage AC-box) is ~29u — so 16u sits in the gap with margin on both sides. Glass is exempt
    ///     (see <see cref="OpacityFilter.IsAlwaysSeeThrough" />): it is see-through by material at any thickness.
    /// </summary>
    private const float SeeThroughMaxThicknessUnits = 16f;

    /// <summary>Collision-attribute tag substrings that mark an <b>invisible</b> clip volume (no vision blocking).</summary>
    private static readonly string[] ClipTagMarkers =
    {
        "playerclip", "player_clip", "npcclip", "npc_clip", "grenadeclip", "grenade_clip",
        "blockbullets", "passbullets", "clip",
    };

    /// <summary>
    ///     Interaction-layer tag substrings that mark a shape as <b>see-through</b> (bullets/vision pass) — CS2's
    ///     own authored layers (<c>InteractionLayers.PassBullets</c> / <c>InteractionLayers.Window</c>). A shape
    ///     carrying one of these (and no vision-blocking layer) is NOT baked as an occluder. These are the game's
    ///     engine layer names, not map or surfaceprop names → generic across every map, zero hardcoding.
    /// </summary>
    private static readonly string[] SeeThroughLayerMarkers = { "passbullets", "window" };

    /// <summary>
    ///     Interaction-layer tag substrings that <b>definitively</b> block vision (<c>InteractionLayers.BlockLos</c>
    ///     / <c>InteractionLayers.CStrikeOpaque</c> / world geometry). Presence force-bakes the shape and overrides
    ///     any see-through hint below — a genuine occluder is never dropped (leak-safe).
    /// </summary>
    private static readonly string[] VisionBlockingLayerMarkers = { "blocklos", "opaque", "worldgeometry" };

    /// <summary>Per-shape vision-opacity as read from the collision attribute's interaction-layer tag set.</summary>
    private enum LayerOpacity : byte
    {
        /// <summary>No decisive layer (empty tag set / clip-only) → defer to today's behavior (surfaceprop fallback).</summary>
        Unknown = 0,

        /// <summary>An authored see-through layer (passbullets/window) and no vision-blocking layer → exclude.</summary>
        SeeThrough = 1,

        /// <summary>An authored vision-blocking layer (blocklos/opaque) → force-bake, overrides fallbacks.</summary>
        VisionBlocking = 2,
    }

    /// <summary>World_physics resource extensions, in preference order (embedded vmdl phys is the fallback).</summary>
    private static readonly string[] WorldPhysicsExts = { "vphys_c", "vmdl_c" };

    /// <summary>
    ///     Load and filter a map's world collision geometry.
    /// </summary>
    /// <param name="vpkPath">Path to a map <c>_dir.vpk</c> (workshop) or a bare map/pak <c>.vpk</c>.</param>
    /// <param name="filter">See-through surface-property filter (opacity mask).</param>
    /// <param name="mapName">
    ///     Current map name. Used to disambiguate which <c>world_physics</c> to bake when a package holds several
    ///     (a multi-map addon vpk, or a stub + the real one): the candidate whose full path matches
    ///     <c>maps/&lt;mapName&gt;</c> is preferred, then the largest packed entry.
    /// </param>
    /// <param name="excludeClips">
    ///     When true, also drop invisible clip shapes (collision-attribute tag match). Default false to honour
    ///     the conservative INCLUDE spec; the clip triangle count is reported regardless.
    /// </param>
    public static CollisionGeometry Load(
        string vpkPath, OpacityFilter filter, string mapName, bool excludeClips = false)
    {
        if (!File.Exists(vpkPath))
            throw new FileNotFoundException($"map vpk not found: {vpkPath}", vpkPath);

        var temps    = new List<string>();
        var packages = new List<Package>();

        try
        {
            var outer      = OpenPackage(vpkPath, packages);
            var candidates = CollectKeyCandidates(outer, mapName);
            if (candidates.Count == 0)
                throw new InvalidDataException(
                    "Could not locate world_physics.{vphys_c|vmdl_c} in package or nested map vpk.");

            // Best first: a maps/<mapName> path match, then the largest packed entry (a stub world_physics is
            // tiny). Identical ordering to TryGetPhysicsKey so the geometry key matches the cache key.
            candidates.Sort(CompareCandidates);

            foreach (var cand in candidates)
            {
                // The bake-cache key is ALWAYS the outer-directory entry (a direct world_physics resource, or the
                // nested map vpk entry) so the cheap key path can read its CRC/size WITHOUT extracting the nested
                // vpk. Here, on the miss path, we extract only THIS one candidate.
                var keyCrc  = cand.Entry.CRC32;
                var keySize = cand.Entry.TotalLength;

                Package      ownerPkg;
                PackageEntry worldEntry;
                string       chosenPath;
                int          worldPhysicsCount;

                if (!cand.IsNested)
                {
                    ownerPkg          = outer;
                    worldEntry        = cand.Entry;
                    chosenPath        = cand.Path;
                    worldPhysicsCount = candidates.Count(c => !c.IsNested);
                }
                else
                {
                    // Extract ONLY this nested map vpk and pick its best inner world_physics (same map+size rule).
                    outer.ReadEntry(cand.Entry, out var bytes);
                    var tmp = NewTemp($"{cand.Entry.FileName}.vpk", temps);
                    File.WriteAllBytes(tmp, bytes);
                    var np = OpenPackage(tmp, packages);

                    var inner = SelectInnerWorldPhysics(np, mapName, out worldPhysicsCount, out var innerPath);
                    if (inner is null)
                        continue; // this nested vpk carried no world_physics → fall through to the next candidate

                    ownerPkg   = np;
                    worldEntry = inner;
                    chosenPath = innerPath;
                }

                ownerPkg.ReadEntry(worldEntry, out var resourceBytes);
                var rtmp = NewTemp($"world_physics.{worldEntry.TypeName}", temps);
                File.WriteAllBytes(rtmp, resourceBytes);

                using var resource = new Resource();
                resource.Read(rtmp);

                var phys = resource.ResourceType switch
                {
                    ResourceType.Model                => ((Model)resource.DataBlock!).GetEmbeddedPhys(),
                    ResourceType.PhysicsCollisionMesh => (PhysAggregateData)resource.DataBlock!,
                    _                                 => null,
                } ?? throw new InvalidDataException(
                    $"No PhysAggregateData for world_physics resource type {resource.ResourceType}");

                var geom = Extract(phys, filter, excludeClips, keyCrc, keySize);
                geom.Stats.ChosenPath                 = chosenPath;
                geom.Stats.KeyCandidateCount          = candidates.Count;
                geom.Stats.WorldPhysicsCandidateCount = worldPhysicsCount;
                return geom;
            }

            throw new InvalidDataException(
                "No world_physics found in any candidate (direct or nested map vpk).");
        }
        finally
        {
            foreach (var p in packages)
                p.Dispose();
            foreach (var f in temps)
            {
                try { File.Delete(f); } catch { /* best-effort */ }
            }
        }
    }

    /// <summary>
    ///     Cheaply resolve a map's <c>world_physics</c> bake-cache key (VPK CRC32 + uncompressed size) WITHOUT
    ///     extracting or parsing any geometry, and — critically — WITHOUT extracting any nested map vpk into RAM.
    ///     The key is the winning outer-directory <see cref="PackageEntry" /> (a direct <c>world_physics</c>
    ///     resource, or the selected nested map <c>.vpk</c> entry), whose CRC/size live in the outer package's
    ///     directory and are readable directly. Uses the SAME candidate selection as <see cref="Load" /> so the
    ///     key it returns matches the key <see cref="Load" /> stamps on its geometry. Never throws (returns false
    ///     on any failure → caller rebuilds).
    /// </summary>
    public static bool TryGetPhysicsKey(string vpkPath, string mapName, out uint crc32, out long size)
    {
        crc32 = 0;
        size  = 0;

        if (!File.Exists(vpkPath))
            return false;

        var packages = new List<Package>();

        try
        {
            var outer      = OpenPackage(vpkPath, packages);
            var candidates = CollectKeyCandidates(outer, mapName);
            if (candidates.Count == 0)
                return false;

            candidates.Sort(CompareCandidates);
            var best = candidates[0]; // nested candidates carry the nested-vpk entry — never extracted here
            crc32 = best.Entry.CRC32;
            size  = best.Entry.TotalLength;
            return true;
        }
        catch
        {
            return false;
        }
        finally
        {
            foreach (var p in packages)
                p.Dispose();
        }
    }

    private static CollisionGeometry Extract(
        PhysAggregateData phys, OpacityFilter filter, bool excludeClips, uint crc32, long size)
    {
        var stats = new LoadStats();

        // Resolve the surface-property name + see-through flag once per property index (O(1) per triangle after).
        var propHashes = SafeSurfacePropertyHashes(phys);
        var propNames     = new string[propHashes.Length];
        var propSeeThru   = new bool[propHashes.Length];
        var propAlwaysSee = new bool[propHashes.Length]; // glass-family: see-through regardless of shape thickness
        for (var i = 0; i < propHashes.Length; i++)
        {
            propNames[i]     = StringToken.GetKnownString(propHashes[i]);
            propSeeThru[i]   = filter.IsSeeThrough(propNames[i]);
            propAlwaysSee[i] = filter.IsAlwaysSeeThrough(propNames[i]);
        }

        // Resolve collision-attribute tag-set (as a stable string) + clip flag + interaction-layer opacity once
        // per attribute index (O(1) per triangle after). The opacity comes from the shape's own interaction
        // layers (m_InteractAsStrings) — the game's generic vision signal, no map/surfaceprop hardcoding.
        var attrs        = SafeCollisionAttributes(phys);
        var attrTagsStr  = new string[attrs.Count];
        var attrIsClip   = new bool[attrs.Count];
        var attrOpacity  = new LayerOpacity[attrs.Count];
        for (var i = 0; i < attrs.Count; i++)
        {
            var tags = SafeTagStrings(attrs[i]);
            attrTagsStr[i] = tags.Count == 0 ? "(none)" : string.Join('+', tags.OrderBy(t => t));
            attrIsClip[i]  = IsClipTagSet(tags);
            attrOpacity[i] = ClassifyLayerOpacity(tags);
        }

        var v0 = new List<Vector3>();
        var v1 = new List<Vector3>();
        var v2 = new List<Vector3>();

        var min = new Vector3(float.PositiveInfinity);
        var max = new Vector3(float.NegativeInfinity);

        var bindPose = phys.BindPose;

        // shapeThin is decided ONCE per collision shape (mesh / hull) from its world-space AABB min dimension, then
        // passed to every triangle of that shape. The see-through NAME is still resolved per-triangle (a mesh can be
        // multi-material via Shape.Materials): a thin shape drops only its see-through-named triangles; a solid-named
        // triangle in the same thin shape still bakes. A THICK shape never drops on name (protects nuke's ducts).
        void Emit(
            Vector3 a, Vector3 b, Vector3 c, int surfIdx, string tagsStr, bool isClip, LayerOpacity opacity,
            bool shapeThin)
        {
            var nameSeeThru = (uint)surfIdx < (uint)propSeeThru.Length && propSeeThru[surfIdx];
            var nameAlways  = (uint)surfIdx < (uint)propAlwaysSee.Length && propAlwaysSee[surfIdx];
            var name        = (uint)surfIdx < (uint)propNames.Length ? propNames[surfIdx] : "(none)";

            // Name is necessary but NOT sufficient: exclude on name only if the shape is geometrically thin
            // (fence/grate/vent/slat), OR the name is glass-family (see-through by material at any thickness).
            var seeThrough = nameAlways || (nameSeeThru && shapeThin);

            Bump(stats.ByCollisionTags, tagsStr);

            // PRIMARY (generic, zero hardcoding): the game's own interaction layers. A shape the engine authored
            // as see-through (passbullets/window, and NOT vision-blocking) is not an occluder — drop it. This is
            // the map-agnostic signal that kills the fence/grate/vent false-hide class wherever it is tagged. Not
            // gated on thinness: an authored passbullets layer is a definitive see-through signal on its own.
            if (opacity == LayerOpacity.SeeThrough)
            {
                stats.ExcludedByOpacityLayer++;
                Bump(stats.ExcludedByOpacityTag, tagsStr);
                Bump(stats.ExcludedByProp, name);
                return;
            }

            // A definitively vision-blocking interaction layer (blocklos/opaque) overrides the surfaceprop-name
            // fallback → a genuine occluder is never dropped even if its surfaceprop name looks see-through.
            var forceOpaque = opacity == LayerOpacity.VisionBlocking;

            // SECONDARY fallback: the surfaceprop-name see-through set, gated on per-shape thinness (above). Catches
            // see-through geometry the compiled physics leaves on the DEFAULT collision attribute with NO interaction
            // tag (e.g. de_mirage metalvent / glass), which the layer test alone cannot distinguish from a wall.
            if (seeThrough && !forceOpaque)
            {
                stats.ExcludedSeeThrough++;
                Bump(stats.ExcludedByProp, name);
                return;
            }

            // Diagnostic: a see-through FAMILY name that we are KEEPING solid — because the shape is thick (nuke
            // duct / mirage AC-box) or force-opaque. This is the leak-guard firing; surface it for future tuning.
            if (nameSeeThru)
            {
                stats.IncludedThickDespiteName++;
                Bump(stats.IncludedThickByProp, name);
            }

            if (excludeClips && isClip)
            {
                stats.ExcludedClip++;
                return;
            }

            v0.Add(a);
            v1.Add(b);
            v2.Add(c);
            stats.IncludedTriangles++;
            Bump(stats.IncludedByProp, name);

            min = Vector3.Min(min, a); max = Vector3.Max(max, a);
            min = Vector3.Min(min, b); max = Vector3.Max(max, b);
            min = Vector3.Min(min, c); max = Vector3.Max(max, c);
        }

        for (var p = 0; p < phys.Parts.Length; p++)
        {
            var shape = phys.Parts[p].Shape;
            var pose  = bindPose.Length == 0 ? Matrix4x4.Identity : bindPose[p];

            stats.SphereShapes  += shape.Spheres.Length;
            stats.CapsuleShapes += shape.Capsules.Length;

            // -- Triangle meshes (bulk of static world collision) --
            foreach (var meshDesc in shape.Meshes)
            {
                var tagsStr = TagsFor(attrTagsStr, meshDesc.CollisionAttributeIndex);
                var isClip  = FlagFor(attrIsClip, meshDesc.CollisionAttributeIndex);
                var opacity = OpacityFor(attrOpacity, meshDesc.CollisionAttributeIndex);

                var verts = meshDesc.Shape.GetVertices();
                var world = new Vector3[verts.Length];
                for (var i = 0; i < verts.Length; i++)
                    world[i] = Vector3.Transform(verts[i], pose);

                var shapeThin = IsShapeThin(world); // once per mesh (world-space AABB min dim ≤ threshold)

                var materials = meshDesc.Shape.Materials; // per-triangle surface-property index (may be empty)
                var t = 0;
                foreach (var tri in meshDesc.Shape.GetTriangles())
                {
                    var surfIdx = materials.Length > 0 ? materials[t] : meshDesc.SurfacePropertyIndex;
                    Emit(world[tri.X], world[tri.Y], world[tri.Z], surfIdx, tagsStr, isClip, opacity, shapeThin);
                    stats.MeshTriangles++;
                    t++;
                }
            }

            // -- Convex hulls (brushes / prop collision): fan-triangulate each face via half-edges --
            foreach (var hullDesc in shape.Hulls)
            {
                var tagsStr = TagsFor(attrTagsStr, hullDesc.CollisionAttributeIndex);
                var isClip  = FlagFor(attrIsClip, hullDesc.CollisionAttributeIndex);
                var opacity = OpacityFor(attrOpacity, hullDesc.CollisionAttributeIndex);
                var surfIdx = hullDesc.SurfacePropertyIndex; // hulls carry a single surface property

                var positions = hullDesc.Shape.GetVertexPositions();
                var world = new Vector3[positions.Length];
                for (var i = 0; i < positions.Length; i++)
                    world[i] = Vector3.Transform(positions[i], pose);

                var shapeThin = IsShapeThin(world); // once per hull (world-space AABB min dim ≤ threshold)

                var faces = hullDesc.Shape.GetFaces();
                var edges = hullDesc.Shape.GetEdges();

                foreach (var face in faces)
                {
                    var startEdge = face.Edge;
                    for (var e = edges[startEdge].Next; e != startEdge;)
                    {
                        var nextEdge = edges[e].Next;
                        if (nextEdge == startEdge)
                            break;

                        Emit(world[edges[startEdge].Origin], world[edges[e].Origin], world[edges[nextEdge].Origin],
                            surfIdx, tagsStr, isClip, opacity, shapeThin);
                        stats.HullTriangles++;

                        e = nextEdge;
                    }
                }
            }
        }

        if (v0.Count == 0)
        {
            min = Vector3.Zero;
            max = Vector3.Zero;
        }

        stats.WorldMin = min;
        stats.WorldMax = max;

        return new CollisionGeometry(v0.ToArray(), v1.ToArray(), v2.ToArray(), crc32, size, stats);
    }

    // ── helpers ──────────────────────────────────────────────────────────────────────────────────────────

    /// <summary>
    ///     Is a collision shape geometrically <b>thin</b> — its world-space AABB minimum dimension ≤
    ///     <see cref="SeeThroughMaxThicknessUnits" />? A thin shape is the geometric signature of a fence / grate /
    ///     vent / slat (flat in one axis); a thick one is a wall / duct / box. Decided once per shape from ALL its
    ///     vertices. <b>Fails toward INCLUDE (returns false = not thin = opaque)</b> on any ambiguity — no verts,
    ///     or a non-finite AABB — so a shape is never dropped as see-through on a computation we could not trust
    ///     (the no-leak direction). A genuinely flat (zero-min-dim) shape is a valid thin result, not ambiguity.
    /// </summary>
    private static bool IsShapeThin(Vector3[] world)
    {
        if (world.Length == 0)
            return false; // no geometry to measure → cannot claim see-through → INCLUDE (opaque)

        var mn = new Vector3(float.PositiveInfinity);
        var mx = new Vector3(float.NegativeInfinity);
        foreach (var w in world)
        {
            mn = Vector3.Min(mn, w);
            mx = Vector3.Max(mx, w);
        }

        var ext = mx - mn;
        if (!float.IsFinite(ext.X) || !float.IsFinite(ext.Y) || !float.IsFinite(ext.Z))
            return false; // NaN/Inf vertex → untrustworthy AABB → INCLUDE (opaque)

        var minDim = MathF.Min(ext.X, MathF.Min(ext.Y, ext.Z));
        return minDim <= SeeThroughMaxThicknessUnits;
    }

    private static string TagsFor(string[] table, int idx)
        => (uint)idx < (uint)table.Length ? table[idx] : "(none)";

    private static bool FlagFor(bool[] table, int idx)
        => (uint)idx < (uint)table.Length && table[idx];

    private static LayerOpacity OpacityFor(LayerOpacity[] table, int idx)
        => (uint)idx < (uint)table.Length ? table[idx] : LayerOpacity.Unknown;

    /// <summary>
    ///     Classify a collision attribute's vision-opacity from its interaction-layer tag set. Vision-blocking
    ///     layers win over see-through (a mixed set errs opaque → no leak); an empty / clip-only / otherwise
    ///     undecided set returns <see cref="LayerOpacity.Unknown" /> so the caller keeps today's behavior.
    /// </summary>
    private static LayerOpacity ClassifyLayerOpacity(IReadOnlyCollection<string> tags)
    {
        if (tags.Count == 0)
            return LayerOpacity.Unknown;

        var seeThrough = false;
        foreach (var tag in tags)
        {
            foreach (var marker in VisionBlockingLayerMarkers)
            {
                if (tag.Contains(marker, StringComparison.OrdinalIgnoreCase))
                    return LayerOpacity.VisionBlocking; // definitive → force bake, overrides any see-through hint
            }

            foreach (var marker in SeeThroughLayerMarkers)
            {
                if (tag.Contains(marker, StringComparison.OrdinalIgnoreCase))
                    seeThrough = true;
            }
        }

        return seeThrough ? LayerOpacity.SeeThrough : LayerOpacity.Unknown;
    }

    private static bool IsClipTagSet(IReadOnlyCollection<string> tags)
    {
        if (tags.Count == 0)
            return false;

        // A shape is treated as an invisible clip only when EVERY one of its tags is a clip-marker (a mixed
        // solid+clip tag-set is kept, erring visible → no leak).
        foreach (var tag in tags)
        {
            var hit = false;
            foreach (var marker in ClipTagMarkers)
            {
                if (tag.Contains(marker, StringComparison.OrdinalIgnoreCase))
                {
                    hit = true;
                    break;
                }
            }

            if (!hit)
                return false;
        }

        return true;
    }

    private static void Bump(Dictionary<string, long> d, string key)
        => d[key] = d.TryGetValue(key, out var v) ? v + 1 : 1;

    private static uint[] SafeSurfacePropertyHashes(PhysAggregateData phys)
    {
        try { return phys.SurfacePropertyHashes ?? Array.Empty<uint>(); }
        catch { return Array.Empty<uint>(); }
    }

    private static IReadOnlyList<KVObject> SafeCollisionAttributes(PhysAggregateData phys)
    {
        try { return phys.CollisionAttributes ?? Array.Empty<KVObject>(); }
        catch { return Array.Empty<KVObject>(); }
    }

    private static IReadOnlyCollection<string> SafeTagStrings(KVObject attr)
    {
        try
        {
            var arr = attr.GetArray<string>("m_InteractAsStrings")
                      ?? attr.GetArray<string>("m_PhysicsTagStrings");
            return arr ?? (IReadOnlyCollection<string>)Array.Empty<string>();
        }
        catch
        {
            return Array.Empty<string>();
        }
    }

    private static Package OpenPackage(string path, List<Package> packages)
    {
        var pkg = new Package();
        pkg.Read(path);
        packages.Add(pkg);
        return pkg;
    }

    private static string NewTemp(string name, List<string> temps)
    {
        var t = Path.Combine(Path.GetTempPath(), $"{Guid.NewGuid():N}_{name}");
        temps.Add(t);
        return t;
    }

    /// <summary>
    ///     A <c>world_physics</c> cache-key candidate discoverable WITHOUT extracting a nested vpk: either a
    ///     direct <c>world_physics</c> resource entry in the outer package, or a nested map <c>.vpk</c> entry
    ///     (recursed into only on a cache miss). <see cref="Entry" />'s CRC32/TotalLength is the bake-cache key.
    /// </summary>
    private readonly struct KeyCandidate
    {
        public readonly PackageEntry Entry;
        public readonly bool         IsNested;
        public readonly string       Path;
        public readonly bool         MatchesMap;

        public KeyCandidate(PackageEntry entry, bool isNested, string path, bool matchesMap)
        {
            Entry      = entry;
            IsNested   = isNested;
            Path       = path;
            MatchesMap = matchesMap;
        }
    }

    // Best first: a maps/<mapName> path match beats a non-match; among equals the larger packed entry wins (a
    // stub world_physics is tiny). Deterministic + identical between TryGetPhysicsKey and Load so their keys agree.
    private static int CompareCandidates(KeyCandidate a, KeyCandidate b)
    {
        if (a.MatchesMap != b.MatchesMap)
            return a.MatchesMap ? -1 : 1;
        return b.Entry.TotalLength.CompareTo(a.Entry.TotalLength);
    }

    private static bool MatchesMap(string fullPath, string mapName)
        => !string.IsNullOrEmpty(mapName) && fullPath.Contains(mapName, StringComparison.OrdinalIgnoreCase);

    /// <summary>
    ///     Collect every cache-key candidate WITHOUT extracting any nested vpk: direct <c>world_physics</c>
    ///     resource entries in the outer package, plus each nested MAP <c>.vpk</c> entry (its inner
    ///     <c>world_physics</c> is parsed only on a cache miss, in <see cref="Load" />). Nested vpks are
    ///     restricted to those living under <c>maps/</c> so a large material/sound addon vpk cannot win the
    ///     largest-size tie-break or inflate the candidate count; if none qualify and there was no direct hit,
    ///     falls back to considering every nested vpk (last resort — <see cref="Load" /> still probes each).
    /// </summary>
    private static List<KeyCandidate> CollectKeyCandidates(Package outer, string mapName)
    {
        var list = new List<KeyCandidate>();
        if (outer.Entries is not { } entries)
            return list;

        foreach (var ext in WorldPhysicsExts)
        {
            if (!entries.TryGetValue(ext, out var byExt))
                continue;
            foreach (var e in byExt)
            {
                if (!string.Equals(e.FileName, "world_physics", StringComparison.OrdinalIgnoreCase))
                    continue;
                var path = e.GetFullPath();
                list.Add(new KeyCandidate(e, false, path, MatchesMap(path, mapName)));
            }
        }

        if (entries.TryGetValue("vpk", out var nestedVpks))
        {
            foreach (var e in nestedVpks)
            {
                var path = e.GetFullPath();
                if (path.Contains("maps", StringComparison.OrdinalIgnoreCase))
                    list.Add(NestedCandidate(e, path, mapName));
            }

            // No direct hit AND no maps/-scoped nested vpk → consider every nested vpk as a last resort.
            if (list.Count == 0)
            {
                foreach (var e in nestedVpks)
                    list.Add(NestedCandidate(e, e.GetFullPath(), mapName));
            }
        }

        return list;
    }

    private static KeyCandidate NestedCandidate(PackageEntry e, string path, string mapName)
    {
        var matches = MatchesMap(path, mapName)
                      || string.Equals(e.FileName, mapName, StringComparison.OrdinalIgnoreCase);
        return new KeyCandidate(e, true, path, matches);
    }

    /// <summary>
    ///     Pick the best <c>world_physics</c> resource inside an (already-extracted) nested vpk: a
    ///     <c>maps/&lt;mapName&gt;</c> path match first, then the largest packed entry. <paramref name="count" />
    ///     reports how many <c>world_physics</c> entries the vpk held (a multi-map tell).
    /// </summary>
    private static PackageEntry? SelectInnerWorldPhysics(
        Package pkg, string mapName, out int count, out string path)
    {
        path  = string.Empty;
        count = 0;

        if (pkg.Entries is not { } entries)
            return null;

        PackageEntry? best      = null;
        var           bestMatch = false;

        foreach (var ext in WorldPhysicsExts)
        {
            if (!entries.TryGetValue(ext, out var byExt))
                continue;
            foreach (var e in byExt)
            {
                if (!string.Equals(e.FileName, "world_physics", StringComparison.OrdinalIgnoreCase))
                    continue;

                count++;
                var p = e.GetFullPath();
                var m = MatchesMap(p, mapName);
                if (best is null
                    || (m && !bestMatch)
                    || (m == bestMatch && e.TotalLength > best.TotalLength))
                {
                    best      = e;
                    bestMatch = m;
                    path      = p;
                }
            }
        }

        return best;
    }
}
