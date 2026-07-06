using System;
using System.Collections.Generic;

namespace FogOfWar.Engine;

/// <summary>
///     Decides which static-collision surfaces are <b>see-through for vision</b> and must NOT be baked as
///     line-of-sight occluders. The physics mesh carries a per-triangle surface-property index (resolved to a
///     name via VRF's <c>StringToken.GetKnownString</c>, MurmurHash2 seed 826366246); this filter maps that
///     name to an include/exclude decision.
///
///     <para><b>Why name-based (and not the map's own data):</b> CS2's <c>world_physics</c> carries NO vision /
///     opacity signal a dedicated server can read — verified empirically: a fence's collision attribute is
///     byte-identical to a solid wall (both <c>CollisionGroupString="Default"</c>, no interaction/opacity
///     layer), and the real "see-through" flag lives only in the client render material (alpha-test), which has
///     no link to the physics geometry and is not shipped in the server content depot. So the surface-property
///     <b>name</b> is the only lever. To keep that from being per-material whack-a-mole, matching is by
///     see-through <b>FAMILY</b>: each keyword below is matched as a case-insensitive <i>substring</i> of the
///     surface-property name, so one keyword covers a whole family (<c>vent</c> → <c>metalvent</c> /
///     <c>metalvent_grate</c>; <c>fence</c> → <c>chainlinkfence</c> / any <c>*fence*</c>) and new fence/grate/
///     vent materials on any map are caught automatically.</para>
///
///     <para><b>Name is necessary, NOT sufficient — geometry decides:</b> the same surfaceprop is ambiguous
///     across official maps. <c>metalvent</c> tags a THIN see-through lattice on de_mirage (min-AABB-dim 1–11u)
///     but SOLID crawl-duct meshes on de_nuke (min-AABB-dim ~346u+, players crawl inside). A pure name filter
///     would carve the nuke ducts open → a permanent wallhack leak in nuke's most sensitive area. So
///     <see cref="IsSeeThrough" /> is only the NAME half of the test: <see cref="MapCollisionLoader" /> ALSO gates
///     on per-shape geometric <b>thinness</b> — a surface is dropped as see-through only if its name matches a
///     family here AND its collision shape is thin. Thick shapes with a see-through name are kept solid.</para>
///
///     <para><b>Glass is the exception:</b> <see cref="IsAlwaysSeeThrough" /> reports the <c>glass</c> family,
///     which is see-through by material regardless of thickness (a glass pane or a glass block both pass vision).
///     The loader excludes those without the thinness gate. All other families require thinness.</para>
///
///     <para><b>Failure bias:</b> a false-HIDE (enemy behind a fence wrongly culled) is gameplay-breaking; an
///     over-exclude (a solid surface dropped) is a wallhack leak. Families lean toward exclusion, kept distinctive
///     to avoid leaks, now backstopped by the thinness gate. Two accepted edges, both documented: <c>glass</c>
///     also matches the rare solid <c>fiberglass</c>; <c>slats</c> (not <c>slat</c>) is used so it never matches
///     solid <c>slate</c>.</para>
///
///     <para>Extra keywords can be added from config (<c>fogOfWar.extraSeeThroughProps</c>) — they are matched
///     as substrings too, so prefer specific terms.</para>
/// </summary>
internal sealed class OpacityFilter
{
    // See-through material FAMILIES — matched as case-insensitive SUBSTRINGS of the surfaceprop name.
    private static readonly string[] DefaultKeywords =
    {
        "chainlink",  // chain-link fence
        "fence",      // any *fence*
        "grate",      // metal grate / catwalk grate
        "grating",    // (separate stem — "grate" is not a substring of "grating")
        "grille",     // metal grille (covers "grill"/"grille")
        "vent",       // metalvent / metalvent_grate  ← the de_mirage fence tag
        "lattice",
        "mesh",       // wire mesh
        "netting",
        "slats",      // window slats / louvers (plural on purpose — never matches solid "slate")
        "louver",
        "glass",      // glass / glassbullet / glasssoft / glassfloor (accepts rare "fiberglass" over-exclude)
    };

    /// <summary>
    ///     See-through families that are see-through by <b>material</b>, independent of shape thickness — the
    ///     loader excludes these WITHOUT the geometric thinness gate that the other families require. Only
    ///     <c>glass</c>: a glass pane and a glass block both pass vision, so a thick glass shape must still drop.
    ///     (Kept minimal — every other family, incl. config extras, is a metal fence/grate/vent/lattice that is
    ///     see-through only because it is physically thin, and MUST stay gated to protect thick look-alikes such
    ///     as de_nuke's <c>metalvent</c> crawl-ducts.)
    /// </summary>
    private static readonly string[] AlwaysSeeThroughKeywords = { "glass" };

    private readonly string[] _keywords;

    public OpacityFilter(IEnumerable<string>? extraSeeThrough = null, bool useDefaults = true)
    {
        var list = new List<string>();

        if (useDefaults)
            list.AddRange(DefaultKeywords);

        if (extraSeeThrough is not null)
        {
            foreach (var s in extraSeeThrough)
            {
                if (!string.IsNullOrWhiteSpace(s))
                    list.Add(s.Trim());
            }
        }

        _keywords = list.ToArray();
    }

    /// <summary>The see-through family keywords this filter matches (for reporting).</summary>
    public IReadOnlyCollection<string> SeeThroughNames => _keywords;

    /// <summary>
    ///     True if a surface property's name contains any see-through family keyword. This is the NAME half of the
    ///     see-through test and is <b>necessary but not sufficient</b>: <see cref="MapCollisionLoader" /> excludes a
    ///     shape only when this is true AND the shape is geometrically thin (or the name is
    ///     <see cref="IsAlwaysSeeThrough" />). Unknown / empty names return false (conservative INCLUDE = opaque).
    /// </summary>
    public bool IsSeeThrough(string? surfacePropName)
    {
        if (string.IsNullOrEmpty(surfacePropName))
            return false;

        foreach (var kw in _keywords)
        {
            if (surfacePropName.Contains(kw, StringComparison.OrdinalIgnoreCase))
                return true;
        }

        return false;
    }

    /// <summary>
    ///     True if a surface property's name is a family that is see-through by material regardless of shape
    ///     thickness (currently only <c>glass</c>). The loader excludes these without the geometric thinness gate.
    ///     Unknown / empty names return false. Callers should treat this as a subset of <see cref="IsSeeThrough" />.
    /// </summary>
    public bool IsAlwaysSeeThrough(string? surfacePropName)
    {
        if (string.IsNullOrEmpty(surfacePropName))
            return false;

        foreach (var kw in AlwaysSeeThroughKeywords)
        {
            if (surfacePropName.Contains(kw, StringComparison.OrdinalIgnoreCase))
                return true;
        }

        return false;
    }
}
