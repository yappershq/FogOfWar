using System;
using System.Numerics;

namespace FogOfWar.Engine;

// ── SMOKE / HE OCCLUSION — the ONLY non-fail-open Fog-of-War feature ──────────────────────────────────────
//
// 1:1 C# port of CS2FOW's smoke_occlusion.cpp/.h (concept by karola3vax, MIT). Pure occlusion math — a live
// CS2 smoke cloud's captured density grid is raymarched to decide whether a sightline is blocked, and an HE
// detonation punches a temporary viewing channel through the smoke.
//
// **ModSharp-FREE by construction.** References only System + System.Numerics + <see cref="LosQuery" /> (itself
// immutable / thread-safe and ModSharp-free). Runs on the visibility WORKER thread. The smoke grids it reads are
// plain copied arrays captured on the game thread (never a live pointer).
//
// **⚠️ NON-FAIL-OPEN — READ THIS.** Every OTHER Fog-of-War degradation errs VISIBLE (a bug can only fail to hide
// a hidden enemy). Smoke is the ONE exception: it HIDES an otherwise-visible player (that is its whole point).
// A false positive here is a gameplay-breaking false-hide, so the capture side (<c>FogOfWarModule.CaptureSmokes</c>)
// MUST drop ALL smoke for a pass on ANY read/validation failure — never feed this a partial or torn grid. The
// finiteness / age / frame guards mirrored here from upstream are the second line of defence.
//
// The constants and thresholds below are copied EXACTLY from upstream — do NOT approximate the volume with a
// sphere or round any threshold; the DDA raymarch through the real density grid is the point.
internal static class SmokeOcclusion
{
    // Grid dimensions (upstream k_smoke_axis_cells / _cell_count / _mask_bytes / _max_*).
    internal const int AxisCells       = 32;
    internal const int CellCount       = AxisCells * AxisCells * AxisCells; // 32768
    internal const int MaskBytes       = CellCount / 8;                     // 4096
    internal const int MaxVolumes      = 32;
    internal const int MaxHeClearances = 64;

    // Occlusion math constants — EXACT upstream values (smoke_occlusion.cpp anonymous namespace).
    private const float HalfExtent        = 320.0f; // k_half_extent
    private const float CellSize          = 20.0f;  // k_cell_size
    private const float DensityScale      = 50.0f;  // k_density_scale
    private const float IgnoreDensity     = 0.1f;   // k_ignore_density
    private const float OpaqueDensity     = 0.8f;   // k_opaque_density
    private const float BlockDensity      = 0.2f;   // k_block_density  (line blocked when total >= 0.2)
    private const float VisualTimingMargin = 0.5f;  // k_visual_timing_margin
    private const int   MaxSteps          = 128;    // k_max_steps

    // ── Public entry: is the (origin→target) sightline blocked by the captured smoke? ────────────────────
    // Faithful port of smoke_line_blocked. Accumulates per-volume density (scaled by the volume's age ramp) and
    // returns true once the total reaches the block threshold. An HE clearance that "opens" a volume skips it.
    // <paramref name="geometry" /> is the SAME <see cref="LosQuery" /> the caller (PairBlocked) already holds — the
    // HE clearance wall-check needs it (null ⇒ HE never opens a volume, matching upstream's nullptr geometry).
    internal static bool LineBlocked(
        SmokeSnapshot snapshot,
        Vector3       origin,
        Vector3       target,
        float         ageAdvanceSeconds,
        LosQuery?     geometry)
    {
        var direction = target - origin;
        if (!Finite(origin) || !Finite(target) || !Finite(direction))
            return false;

        var advance = MathF.Max(ageAdvanceSeconds, 0.0f);

        var total = 0.0f;
        var volumes = snapshot.Volumes;
        for (var v = 0; v < volumes.Length; v++)
        {
            ref readonly var volume = ref volumes[v];

            var cleared = false;
            var clearances = snapshot.HeClearances;
            for (var i = 0; i < clearances.Length; i++)
            {
                if (ClearanceOpensVolume(in clearances[i], in volume, origin, target,
                        snapshot.HeClearRadiusUnits, snapshot.HeClearSeconds, ageAdvanceSeconds, geometry))
                {
                    cleared = true;
                    break;
                }
            }

            if (cleared)
                continue;

            total += VolumeDensity(in volume, origin, target) * AgeScale(volume.AgeSeconds + advance);
            if (total >= BlockDensity)
                return true;
        }

        return false;
    }

    // ── Occlusion math (all private static, 1:1 from smoke_occlusion.cpp) ─────────────────────────────────

    private static bool Finite(Vector3 value)
        => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);

    private static float Smoothstep(float value)
    {
        value = Math.Clamp(value, 0.0f, 1.0f);
        return value * value * (3.0f - 2.0f * value);
    }

    // Age ramp: 0 → 1 as the puff grows in (~0.6s onward), back to 0 as it fades out (~16.5s onward). Exact
    // upstream formula (note the ±0.5 visual-timing margin baked into the two smoothstep windows).
    private static float AgeScale(float age)
    {
        var delayedGrowthAge = age - VisualTimingMargin;
        var earlyFadeAge     = age + VisualTimingMargin;
        return Smoothstep((delayedGrowthAge - 0.1f) / 1.4f) * Smoothstep((22.0f - earlyFadeAge) / 5.0f);
    }

    // 5-bit Morton (Z-order) interleave — indexes the density/opaque grid exactly as the engine stores it.
    private static uint MortonIndex(uint x, uint y, uint z)
    {
        uint result = 0;
        for (var bit = 0; bit < 5; bit++)
        {
            result |= ((x >> bit) & 1u) << (bit * 3);
            result |= ((y >> bit) & 1u) << (bit * 3 + 1);
            result |= ((z >> bit) & 1u) << (bit * 3 + 2);
        }

        return result;
    }

    // Slab clip of one axis against [minimum, maximum]. Updates the running [first, last] segment interval.
    private static bool ClipAxis(float origin, float direction, float minimum, float maximum, ref float first, ref float last)
    {
        if (MathF.Abs(direction) < 1.0e-7f)
            return origin >= minimum && origin <= maximum;

        var left  = (minimum - origin) / direction;
        var right = (maximum - origin) / direction;
        if (left > right)
            (left, right) = (right, left);

        first = MathF.Max(first, left);
        last  = MathF.Min(last, right);
        return first <= last;
    }

    // Clip the segment to the volume's 640-unit AABB (2 * HalfExtent). Short-circuit && matches upstream so a
    // failed earlier axis leaves first/last as upstream leaves them.
    private static bool ClipToVolume(in SmokeVolume volume, Vector3 origin, Vector3 direction, ref float first, ref float last)
    {
        var c = volume.Center;
        return ClipAxis(origin.X, direction.X, c.X - HalfExtent, c.X + HalfExtent, ref first, ref last)
               && ClipAxis(origin.Y, direction.Y, c.Y - HalfExtent, c.Y + HalfExtent, ref first, ref last)
               && ClipAxis(origin.Z, direction.Z, c.Z - HalfExtent, c.Z + HalfExtent, ref first, ref last);
    }

    private static int CellCoordinate(float value, float minimum)
        => Math.Clamp((int) MathF.Floor((value - minimum) / CellSize), 0, AxisCells - 1);

    // 3D-DDA raymarch through the volume's density grid. Returns 1.0 on a hard-opaque cell (opaque bit or density
    // >= OpaqueDensity) or once the accumulated density crosses BlockDensity; otherwise the accumulated density.
    private static float VolumeDensity(in SmokeVolume volume, Vector3 origin, Vector3 target)
    {
        var direction = target - origin;
        var first = 0.0f;
        var last  = 1.0f;
        if (!ClipToVolume(in volume, origin, direction, ref first, ref last))
            return 0.0f;

        var entry = Math.Clamp(first, 0.0f, 1.0f);
        var exit  = Math.Clamp(last, 0.0f, 1.0f);
        if (entry > exit)
            return 0.0f;

        var minimum = new Vector3(volume.Center.X - HalfExtent, volume.Center.Y - HalfExtent, volume.Center.Z - HalfExtent);
        var start   = origin + direction * entry;
        var finish  = origin + direction * exit;

        var startParameter = entry;
        if (entry > 0.0f)
        {
            var length = MathF.Sqrt((direction.X * direction.X) + (direction.Y * direction.Y) + (direction.Z * direction.Z));
            if (length > 0.0f)
            {
                var inward = MathF.Min(1.0f / length, MathF.Max(exit - entry, 0.0f));
                startParameter += inward;
                start          += direction * inward;
            }
        }

        Span<int> cell = stackalloc int[3]
        {
            CellCoordinate(start.X, minimum.X),
            CellCoordinate(start.Y, minimum.Y),
            CellCoordinate(start.Z, minimum.Z),
        };
        Span<int> end = stackalloc int[3]
        {
            CellCoordinate(finish.X, minimum.X),
            CellCoordinate(finish.Y, minimum.Y),
            CellCoordinate(finish.Z, minimum.Z),
        };
        Span<float> startValues   = stackalloc float[3] { start.X, start.Y, start.Z };
        Span<float> directions    = stackalloc float[3] { direction.X, direction.Y, direction.Z };
        Span<float> minimumValues = stackalloc float[3] { minimum.X, minimum.Y, minimum.Z };
        Span<int>   step          = stackalloc int[3];
        Span<float> next          = stackalloc float[3];
        Span<float> delta         = stackalloc float[3];

        for (var axis = 0; axis < 3; axis++)
        {
            if (directions[axis] > 0.0f)
            {
                step[axis]  = 1;
                next[axis]  = startParameter + (minimumValues[axis] + ((cell[axis] + 1) * CellSize) - startValues[axis]) / directions[axis];
                delta[axis] = CellSize / directions[axis];
            }
            else if (directions[axis] < 0.0f)
            {
                step[axis]  = -1;
                next[axis]  = startParameter + (minimumValues[axis] + (cell[axis] * CellSize) - startValues[axis]) / directions[axis];
                delta[axis] = -CellSize / directions[axis];
            }
            else
            {
                next[axis]  = float.PositiveInfinity;
                delta[axis] = float.PositiveInfinity;
            }
        }

        var accumulated = 0.0f;
        for (var visited = 0; visited < MaxSteps; visited++)
        {
            if (cell[0] < 0 || cell[1] < 0 || cell[2] < 0
                || cell[0] >= AxisCells || cell[1] >= AxisCells || cell[2] >= AxisCells)
                break;

            var index = MortonIndex((uint) cell[0], (uint) cell[1], (uint) cell[2]);
            if ((volume.Opaque[index >> 3] & (byte) (1u << (int) (index & 7u))) != 0)
                return 1.0f;

            var density = Math.Clamp(volume.Density[index] / DensityScale, 0.0f, 1.0f);
            if (density >= OpaqueDensity)
                return 1.0f;

            if (density > IgnoreDensity)
            {
                accumulated += density;
                if (accumulated >= BlockDensity)
                    return 1.0f;
            }

            if (cell[0] == end[0] && cell[1] == end[1] && cell[2] == end[2])
                break;

            var stepAxis = next[1] <= next[0] ? (next[1] <= next[2] ? 1 : 2) : (next[0] <= next[2] ? 0 : 2);
            if (next[stepAxis] > exit)
                break;

            cell[stepAxis] += step[stepAxis];
            next[stepAxis] += delta[stepAxis];
        }

        return accumulated;
    }

    private static float Square(float value) => value * value;

    // An HE clearance "opens" a volume for this sightline when the detonation is inside the clearance radius of the
    // volume's box, the sightline passes within the radius of the detonation point, and the world does NOT block the
    // detonation→closest-point segment (so the channel is real, not through a wall). 1:1 from upstream.
    private static bool ClearanceOpensVolume(
        in HeClearance clearance,
        in SmokeVolume volume,
        Vector3        origin,
        Vector3        target,
        float          radius,
        float          duration,
        float          ageAdvance,
        LosQuery?      geometry)
    {
        var age = clearance.AgeSeconds + MathF.Max(ageAdvance, 0.0f);
        if (geometry is null || radius <= 0.0f || duration <= 0.0f || age < 0.0f || age >= duration)
            return false;

        if (!float.IsFinite(clearance.DetonationTime) || !float.IsFinite(volume.StartTime)
            || clearance.DetonationTime < volume.StartTime)
            return false;

        var boxDx = MathF.Max(MathF.Abs(clearance.Center.X - volume.Center.X) - HalfExtent, 0.0f);
        var boxDy = MathF.Max(MathF.Abs(clearance.Center.Y - volume.Center.Y) - HalfExtent, 0.0f);
        var boxDz = MathF.Max(MathF.Abs(clearance.Center.Z - volume.Center.Z) - HalfExtent, 0.0f);
        var radiusSquared = Square(radius);
        if (Square(boxDx) + Square(boxDy) + Square(boxDz) > radiusSquared)
            return false;

        var direction     = target - origin;
        var lengthSquared = Square(direction.X) + Square(direction.Y) + Square(direction.Z);
        var fromOrigin    = clearance.Center - origin;
        var parameter = lengthSquared <= 0.0f
            ? 0.0f
            : Math.Clamp(
                ((fromOrigin.X * direction.X) + (fromOrigin.Y * direction.Y) + (fromOrigin.Z * direction.Z)) / lengthSquared,
                0.0f, 1.0f);
        var closest = origin + direction * parameter;
        if (Square(clearance.Center.X - closest.X) + Square(clearance.Center.Y - closest.Y)
            + Square(clearance.Center.Z - closest.Z) > radiusSquared)
            return false;

        // segment_blocked(*geometry, clearance.center, closest) → LosQuery.IsBlocked (static-world BVH).
        return !geometry.IsBlocked(clearance.Center, closest);
    }
}

// ── Immutable captured smoke data (plain copies handed game thread → worker; no live pointers) ─────────────

/// <summary>
///     One captured smoke volume: the cloud center, its age, the frame's density grid and per-cell opaque mask.
///     <see cref="Opaque" /> is <see cref="SmokeOcclusion.MaskBytes" /> (4096) bytes; <see cref="Density" /> is
///     <see cref="SmokeOcclusion.CellCount" /> (32768) floats. The arrays are pooled/owned by the capture side;
///     this struct only borrows them for the immutable snapshot handed to the worker.
/// </summary>
internal readonly struct SmokeVolume
{
    public readonly Vector3 Center;
    public readonly float   AgeSeconds;
    public readonly float   StartTime;
    public readonly byte[]  Opaque;  // 4096
    public readonly float[] Density; // 32768

    public SmokeVolume(Vector3 center, float ageSeconds, float startTime, byte[] opaque, float[] density)
    {
        Center     = center;
        AgeSeconds = ageSeconds;
        StartTime  = startTime;
        Opaque     = opaque;
        Density    = density;
    }
}

/// <summary>One live HE detonation channel: the detonation center, its age this pass, and its detonation time.</summary>
internal readonly struct HeClearance
{
    public readonly Vector3 Center;
    public readonly float   AgeSeconds;
    public readonly float   DetonationTime;

    public HeClearance(Vector3 center, float ageSeconds, float detonationTime)
    {
        Center         = center;
        AgeSeconds     = ageSeconds;
        DetonationTime = detonationTime;
    }
}

/// <summary>
///     Immutable per-pass smoke snapshot handed game thread → worker. <see cref="Volumes" /> and
///     <see cref="HeClearances" /> hold ONLY the active entries (their lengths ARE the counts). A reference type so
///     <c>SmokeSnapshot?</c> is a nullable reference (null ⇒ no smoke this pass → zero occlusion cost).
/// </summary>
internal sealed class SmokeSnapshot
{
    public readonly SmokeVolume[] Volumes;
    public readonly HeClearance[] HeClearances;
    public readonly float         HeClearRadiusUnits;
    public readonly float         HeClearSeconds;

    public SmokeSnapshot(SmokeVolume[] volumes, HeClearance[] heClearances, float heClearRadiusUnits, float heClearSeconds)
    {
        Volumes            = volumes;
        HeClearances       = heClearances;
        HeClearRadiusUnits = heClearRadiusUnits;
        HeClearSeconds     = heClearSeconds;
    }
}
