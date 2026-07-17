using System;
using System.Numerics;
using System.Threading.Tasks;

namespace FogOfWar.Engine;

/// <summary>
///     Pure, thread-safe visibility sampler for the Fog-of-War worker. Given a per-frame snapshot of player
///     state (captured on the game thread, handed over as plain <see cref="System.Numerics.Vector3" /> structs)
///     and an immutable <see cref="LosQuery" /> baked from the map's static collision, it computes the full
///     "is this enemy pair line-of-sight blocked" matrix.
///
///     <para><b>Ray model — every group is ADDITIVE / fail-open by construction.</b> The receiver's REAL eye
///     <c>o1</c> ALWAYS samples every one of the 15 NOW-box target points (4 top corners, center, upper, all 4
///     bottom corners, 4 mid-height edge columns, 1 low-center) — the immovable baseline. Every other origin and
///     point group only ever APPENDS rays on top of that baseline:</para>
///     <list type="bullet">
///       <item>o2 — horizontal peek-predicted eye (A4: keeps the longest unblocked fraction, never a hard
///             collapse) — samples the 8 shared points + the 4 mid-edge points.</item>
///       <item>o3 — vertical peek-predicted eye (jump / drop-off; A5: vertical-only fallback) — samples all 8
///             shared points (A8), when used.</item>
///       <item>o4 — un-crouch eye (A7, ducked observer) — samples the 8 shared points, when used.</item>
///       <item>o5..o8 — RTT-scaled left/right shoulder peeks (upstream shoulder origins) off the real eye AND the
///             peek-predicted eye — each samples the 8 shared points; a shoulder that collapses onto its base eye
///             (offset 0, or reverted through a wall by safe_origin) is skipped as a duplicate.</item>
///       <item>A3 — future-box leading corners (target strafing) — sampled from o1, when the box travels clear.
///             The NOW-box points are NEVER relocated (that relocation was the false-hide bug).</item>
///       <item>A6 — vertical-predicted corners (target jumping / falling) — sampled from o1 AND o2, when the
///             target has real vertical speed.</item>
///       <item>muzzle — the barrel point of the target's active gun (upstream muzzle sampling), sampled from o1
///             AND o2 when the target holds a gun (a peeker who sees the barrel sees the player).</item>
///     </list>
///
///     <para><b>Fail-open / monotone.</b> The result is <c>blocked == !(∃ clear ray)</c>. Because o1 samples a
///     SUPERSET of every point any other origin samples, and the now-box points are never moved, adding any
///     origin / point group can only make "no clear ray" HARDER to reach — it can convert a previously-hidden
///     pair to visible but can NEVER hide a pair that was visible. Adding rays is always safe (never a
///     false-hide / leak).</para>
///
///     <para><b>ModSharp-free by construction.</b> This type references only <see cref="System" /> +
///     <see cref="System.Numerics" /> + <see cref="LosQuery" /> (itself immutable / thread-safe). It NEVER
///     touches a live entity, manager, or any game-thread state, so it is safe to run continuously on a worker
///     thread off the game loop. All inputs are copied value types; the only shared reference is the immutable
///     <see cref="LosQuery" />.</para>
/// </summary>
internal static class LosSampler
{
    /// <summary>Max player slots (matches the module's flat pair matrix stride).</summary>
    public const int Slots = 64;

    // ── Static NOW-box sample points (built by BuildTargetPoints, ALWAYS on the now-box) ──────────────────
    // Layout (indices into the staticPts span):
    //   0..3  : 4 top corners        (z = bmax.Z)
    //   4     : center               (mid-Z)
    //   5     : "upper"              (0.75 height)
    //   6,7   : bottom corners       (first diagonal, z = floor)
    //   8,9   : bottom corners       (other diagonal, z = floor)
    //   10..13: mid-height edge cols (A1 — the 4 vertical edges at mid-Z; the biggest false-hide hole)
    //   14    : low-center           (A2 — 25% height, knee/waist-slit visibility)
    private const int StaticPoints  = 15;
    private const int SharedPoints  = 8;  // P0..P7 — the subset the peek origins (o2/o3/o4) sample
    private const int MidEdgeStart  = 10; // P10..P13 — the A1 mid-height edge columns
    private const int MidEdgePoints = 4;
    private const int O2Points      = SharedPoints + MidEdgePoints; // o2 samples P0..P7 + P10..P13 = 12

    // A3 future-box leading corners (target strafing) and A6 vertical-predicted corners (target jumping /
    // falling) — extra target points. A3 corners are sampled from o1; A6 corners from o1 AND o2.
    private const int A3FutureCorners   = 8; // 4 top + 4 bottom corners of the predicted future box (from o1)
    private const int A6VerticalCorners = 4; // 4 XY corners at the predicted rising/falling Z (from o1 AND o2)

    // RTT-scaled shoulder origins (upstream "left/right shoulder" model): o5/o6 = eye ± lateral, o7/o8 = the
    // peek-predicted eye ± lateral. Each samples the 8 shared points. Additive → fail-open (safe_origin reverts a
    // shoulder that would peek from inside geometry back to its base eye; duplicates are skipped).
    private const int ShoulderOrigins = 4;

    // Held-weapon muzzle target point (upstream muzzle sampling): the barrel of the target's active gun sticks out
    // past the body, so a peeker who can see the muzzle can see the player. Added as ONE extra target point sampled
    // from o1 AND o2 (the two most likely eyes to catch it) → 2 rays. Additive: a gun-barrel point can only reveal.
    private const int MuzzleRays = 2;

    // Worst-case ray count (stackalloc scratch): o1×15 + o2×12 + o3×8 + o4×8 + 4 shoulders×8 + A3×8 + A6×(4×2)
    // + muzzle×2 = 93. Always < 255, so a byte temporal-coherence hint index is never ambiguous with NoClearHint.
    private const int MaxRays =
        StaticPoints + O2Points + SharedPoints /*o3*/ + SharedPoints /*o4*/ + (ShoulderOrigins * SharedPoints)
        + A3FutureCorners + (A6VerticalCorners * 2) + MuzzleRays;

    // Temporal-coherence hint sentinel: "no ray is remembered as last-clearing this pair" (pair fully blocked, or
    // never sampled). Any value >= the pass's ray count is treated as absent, so 0xFF is always out of range.
    public const byte NoClearHint = 0xFF;

    // Upstream sampling constants: k_bounds_inflate=4, k_min_prediction_speed=1. (k_max_prediction_speed is
    // the configurable maxPredSpeed argument.)
    private const float BoundsInflate = 4.0f;
    private const float MinPredSpeed  = 1.0f;

    // Bottom target sample points clamp to origin.Z + this small epsilon (never inflated BELOW the floor) so a
    // feet-visible-through-a-gap enemy (de_nuke trucks) is not wrongly hidden.
    private const float FloorEpsilon = 1.0f;

    // A7: un-crouching raises the eye from the fully-crouched view height to standing (CS2 ~46 → ~64 units). A
    // ducked observer has velocity ≈ 0 (o2/o3 never fire), so this is the ONLY lookahead an un-crouch peek gets.
    // Added as an extra origin o4 (never relocates o1) → fail-open like the rest.
    private const float UnCrouchEyeRise = 18.0f;

    // Shoulder-peek geometry (RTT-scaled origins). DegToRad builds the eye's right vector from the observer yaw;
    // a shoulder that lands within SamePointEpsSq of its base eye (offset clamped to 0, or reverted through a wall)
    // is a pure duplicate and skipped. SamePointEpsSq == upstream k_same_point_epsilon_sq.
    private const float DegToRad       = 0.017453292519943295f;
    private const float SamePointEpsSq = 1.0e-4f;

    // Muzzle-point geometry (upstream k_muzzle_z / k_standing_player_height / k_pelvis_height). The barrel local point
    // is (length, 0, MuzzleZ) in the target's frame; MuzzleZ is scaled for a crouched target via AdjustedLocalZ.
    private const float MuzzleZ              = 60.0f;
    private const float StandingPlayerHeight = 72.0f;
    private const float PelvisHeight         = 38.0f;

    // CS2 team numbering (Sharp.Shared.Enums.CStrikeTeam: TE=2, CT=3). The engine stays ModSharp-free by
    // comparing the raw int the module copies in via (int)team — only strictly-opposing T-vs-CT pairs sample.
    private const int TeamT  = 2;
    private const int TeamCT = 3;

    /// <summary>
    ///     Plain, copied per-player state handed game-thread → worker. Contains no ModSharp types and no live
    ///     references — every field is a copied value captured on the game thread.
    /// </summary>
    internal struct SampledPlayer
    {
        public bool    Valid;    // controller connected + pawn alive this snapshot
        public bool    Human;    // controlling client is a real human (receiver must be human)
        public bool    Ducked;   // observer is fully crouched this snapshot (A7 — enables the un-crouch origin o4)
        public int     Team;     // (int)CStrikeTeam
        public Vector3 Eye;      // observer origin (receiver eye)
        public Vector3 Origin;   // target abs origin (sender)
        public Vector3 Velocity;
        public Vector3 Mins;     // local collision mins
        public Vector3 Maxs;     // local collision maxs
        public float   Yaw;      // eye yaw (degrees) — observer shoulder origins AND target muzzle-point direction
        public float   Rtt;      // observer round-trip time (seconds, from m_iPing) — scales lookahead + shoulder width
        public ushort  WeaponItemDef; // target's active-weapon item-definition index (0 = none/knife → no muzzle point)
    }

    /// <summary>
    ///     Latency-aware sampling tuning (upstream <c>visibility_tuning</c>). Effective per-receiver lookahead grows
    ///     with the receiver's RTT (a high-ping peeker leads further, matching what that client actually sees), and
    ///     the lateral shoulder-peek origins widen with RTT too. All in the receiver's frame — the whole pair
    ///     computation for receiver R uses R's RTT. Prediction is still additive (o1 baseline immovable), so any
    ///     lookahead / shoulder value only ever converts hidden→visible — never a false-hide.
    /// </summary>
    internal readonly struct SamplerTuning
    {
        public readonly float BaseLookaheadSeconds; // lookahead at 0 ping (upstream base_lookahead_ms/1000 = 0.075)
        public readonly float RttLookaheadScale;    // ms of extra lookahead per ms of RTT (ratio on rtt_ms; upstream 1.5)
        public readonly float MaxLookaheadSeconds;  // hard cap (upstream max_lookahead_ms/1000 = 0.375); 0 ⇒ no prediction
        public readonly float PeekMargin;           // minimum predicted-step length (units)
        public readonly float MaxPredSpeed;         // prediction speed cap (units/s)
        public readonly float ShoulderBaseUnits;    // shoulder offset at 0 ping (upstream 24); 0 ⇒ shoulders off
        public readonly float ShoulderRttScale;     // shoulder units per ms of RTT (upstream 0.64)
        public readonly float MaxShoulderUnits;     // shoulder offset cap (upstream 128)

        public SamplerTuning(
            float baseLookaheadSeconds, float rttLookaheadScale, float maxLookaheadSeconds, float peekMargin,
            float maxPredSpeed, float shoulderBaseUnits, float shoulderRttScale, float maxShoulderUnits)
        {
            BaseLookaheadSeconds = baseLookaheadSeconds;
            RttLookaheadScale    = rttLookaheadScale;
            MaxLookaheadSeconds  = maxLookaheadSeconds;
            PeekMargin           = peekMargin;
            MaxPredSpeed         = maxPredSpeed;
            ShoulderBaseUnits    = shoulderBaseUnits;
            ShoulderRttScale     = shoulderRttScale;
            MaxShoulderUnits     = maxShoulderUnits;
        }
    }

    /// <summary>
    ///     upstream <c>visibility_effective_lookahead_seconds</c>: the receiver's effective lookahead grows linearly
    ///     with its RTT — <c>base_ms + rtt_ms · scale</c>, clamped to <c>[0, max_ms]</c>. A <c>MaxLookaheadSeconds</c>
    ///     of 0 disables prediction entirely.
    /// </summary>
    public static float EffectiveLookahead(float rttSeconds, in SamplerTuning t)
    {
        if (t.MaxLookaheadSeconds <= 0f)
            return 0f;
        var rttMs    = MathF.Max(0f, rttSeconds) * 1000f;
        var wantedMs = (t.BaseLookaheadSeconds * 1000f) + (rttMs * MathF.Max(0f, t.RttLookaheadScale));
        return Math.Clamp(wantedMs, 0f, t.MaxLookaheadSeconds * 1000f) / 1000f;
    }

    /// <summary>
    ///     upstream <c>visibility_shoulder_offset_units</c>: lateral shoulder-peek offset in units, scaling from
    ///     <c>ShoulderBaseUnits</c> at 0 ping up to <c>MaxShoulderUnits</c> with RTT. Every player gets at least the
    ///     base offset (a static ±base shoulder peek even at 0 ping); a base of 0 turns shoulders off.
    /// </summary>
    public static float ShoulderOffset(float rttSeconds, in SamplerTuning t)
    {
        // A base of 0 turns shoulder origins OFF entirely (the documented off-switch) — otherwise the RTT term
        // would keep them active for any pinged player. Non-zero base guarantees at least ±base even at 0 ping.
        if (t.ShoulderBaseUnits <= 0f)
            return 0f;
        var baseU   = t.ShoulderBaseUnits;
        var maxU    = MathF.Max(baseU, t.MaxShoulderUnits);
        var wanted  = MathF.Max(0f, rttSeconds) * 1000f * MathF.Max(0f, t.ShoulderRttScale);
        return Math.Clamp(wanted, baseU, maxU);
    }

    /// <summary>
    ///     Recompute the full blocked matrix into <paramref name="blocked" /> (flat
    ///     <c>receiverSlot * Slots + senderSlot</c>, length <c>Slots*Slots</c>, assumed all-false on entry). Only
    ///     alive-human receiver vs alive strictly-opposing sender pairs are sampled; every other cell stays
    ///     <c>false</c> (visible). A cell is set <c>true</c> only when ALL rays are occluded.
    ///
    ///     <para><paramref name="lastClear" /> (optional, length <c>Slots*Slots</c>) is a worker-private temporal-
    ///     coherence hint buffer: per pair it holds the ray index that last cleared LOS, so that ray is tested
    ///     FIRST next pass. Because the per-pair result is an OR-of-clears over the pass's ray set, reordering the
    ///     sweep cannot change the boolean — this is a pure early-out speedup, never a correctness change. Pass
    ///     <c>null</c> to disable. The caller must seed it with <see cref="NoClearHint" /> and reset it on a map
    ///     change.</para>
    /// </summary>
    public static void ComputeMatrix(
        SampledPlayer[]   players,
        int               max,
        LosQuery          los,
        in SamplerTuning  tuning,
        bool              filterTeammates,
        SmokeSnapshot?    smoke,
        float             ageAdvance,
        bool[]            blocked,
        byte[]?           lastClear = null)
    {
        for (var r = 0; r < max; r++)
        {
            ref var rs = ref players[r];
            if (!rs.Valid || !rs.Human)
                continue;

            // Precompute the receiver-only origins ONCE for this whole row — reused across every sender.
            var geom = BuildReceiverGeom(in rs, los, in tuning);

            for (var s = 0; s < max; s++)
            {
                if (s == r)
                    continue;

                ref var ss = ref players[s];
                if (!ss.Valid || !ShouldCull(rs.Team, ss.Team, filterTeammates))
                    continue;

                var pairIdx = (r * Slots) + s;
                blocked[pairIdx] =
                    PairBlocked(in geom, in ss, los, smoke, ageAdvance, lastClear, pairIdx);
            }
        }
    }

    /// <summary>
    ///     Parallel <see cref="ComputeMatrix" /> for high player counts (32–64 slots). The enemy-pair matrix is
    ///     embarrassingly parallel — each receiver row <c>r</c> writes ONLY its own disjoint slice of
    ///     <paramref name="blocked" /> (<c>r*Slots + s</c>) AND its own disjoint slice of the optional
    ///     <paramref name="lastClear" /> hint buffer (<c>r*Slots .. r*Slots+Slots-1</c> — exactly one 64-byte line
    ///     per row), and <see cref="PairBlocked" /> only reads the immutable <paramref name="los" /> + stack-local
    ///     scratch. So the temporal-coherence cache IS race-free under partitioning (no two rows ever touch the same
    ///     hint index), and the result is boolean-identical to the serial <see cref="ComputeMatrix" /> — order of
    ///     evaluation does not affect any cell.
    ///
    ///     <para><paramref name="maxDegreeOfParallelism" /> ≤ 0 uses the scheduler default
    ///     (<see cref="Environment.ProcessorCount" />). On a co-tenant game host, CAP it (2–4) so a 64-player burst
    ///     never starves the game thread or neighbouring servers.</para>
    /// </summary>
    public static void ComputeMatrixParallel(
        SampledPlayer[]   players,
        int               max,
        LosQuery          los,
        SamplerTuning     tuning,
        bool              filterTeammates,
        SmokeSnapshot?    smoke,
        float             ageAdvance,
        bool[]            blocked,
        byte[]?           lastClear              = null,
        int               maxDegreeOfParallelism = -1)
    {
        var options = new ParallelOptions();
        if (maxDegreeOfParallelism > 0)
            options.MaxDegreeOfParallelism = maxDegreeOfParallelism;

        Parallel.For(0, max, options, r =>
        {
            if (!players[r].Valid || !players[r].Human)
                return;

            // Precompute the receiver-only origins ONCE for this row. lastClear (if given) is safe to use here: row r
            // touches only hint indices r*Slots..r*Slots+Slots-1 — disjoint from every other row — so there is no
            // cross-thread write to the same index.
            var geom = BuildReceiverGeom(in players[r], los, in tuning);

            for (var s = 0; s < max; s++)
            {
                if (s == r)
                    continue;

                if (!players[s].Valid || !ShouldCull(players[r].Team, players[s].Team, filterTeammates))
                    continue;

                blocked[(r * Slots) + s] =
                    PairBlocked(in geom, in players[s], los, smoke, ageAdvance, lastClear, (r * Slots) + s);
            }
        });
    }

    /// <summary>Precomputed receiver-only sampling geometry (see <see cref="BuildReceiverGeom" />).</summary>
    private readonly struct ReceiverGeom
    {
        public readonly float   Lookahead;    // effective (RTT-scaled) lookahead — also drives target prediction
        public readonly float   PeekMargin;
        public readonly float   MaxPredSpeed;
        public readonly Vector3 O1;           // real eye (immovable baseline)
        public readonly Vector3 O2;           // horizontal peek-predicted eye
        public readonly bool    O2IsO1;       // o2 EXACTLY collapsed onto o1 (stationary / blocked peek) → dedupe o2 rays
        public readonly bool    UseO3;
        public readonly Vector3 O3;
        public readonly bool    UseO4;
        public readonly Vector3 O4;
        public readonly Vector3 O5, O6, O7, O8; // shoulder peeks (o5/o6 off eye, o7/o8 off predicted eye)

        public ReceiverGeom(
            float lookahead, float peekMargin, float maxPredSpeed, Vector3 o1, Vector3 o2, bool o2IsO1,
            bool useO3, Vector3 o3, bool useO4, Vector3 o4, Vector3 o5, Vector3 o6, Vector3 o7, Vector3 o8)
        {
            Lookahead    = lookahead;
            PeekMargin   = peekMargin;
            MaxPredSpeed = maxPredSpeed;
            O1           = o1;
            O2           = o2;
            O2IsO1       = o2IsO1;
            UseO3        = useO3;
            O3           = o3;
            UseO4        = useO4;
            O4           = o4;
            O5           = o5;
            O6           = o6;
            O7           = o7;
            O8           = o8;
        }
    }

    /// <summary>
    ///     Precompute the receiver-only sampling geometry ONCE per receiver row: the effective (RTT-scaled) lookahead
    ///     and every observer origin (o1 real eye, o2 horizontal-peek, o3 vertical-peek, o4 un-crouch, o5..o8 shoulder
    ///     peeks). All of this — including up to ~10 BVH setup queries — depends ONLY on the receiver, so recomputing
    ///     it for every sender was pure waste; <see cref="PairBlocked" /> now consumes the shared result. The values
    ///     are byte-identical to the old per-pair computation, so the matrix is unchanged.
    /// </summary>
    private static ReceiverGeom BuildReceiverGeom(in SampledPlayer rs, LosQuery los, in SamplerTuning tuning)
    {
        var lookaheadSeconds = EffectiveLookahead(rs.Rtt, tuning);
        var peekMargin       = tuning.PeekMargin;
        var maxPredSpeed     = tuning.MaxPredSpeed;

        // o1 = real eye (immovable baseline). o2 = horizontally peek-predicted eye — A4 keeps the LONGEST unblocked
        // fraction of the predicted step (o1 rays never lost → fail-open); if every step is blocked o2 stays o1.
        var o1      = rs.Eye;
        var predObs = PredictionOffset(rs.Velocity, lookaheadSeconds, peekMargin, maxPredSpeed);

        var o2 = o1;
        if (predObs.X != 0f || predObs.Y != 0f)
        {
            if (!los.IsBlocked(o1, o1 + predObs))
                o2 = o1 + predObs;
            else if (!los.IsBlocked(o1, o1 + (predObs * 0.5f)))
                o2 = o1 + (predObs * 0.5f);
            else if (!los.IsBlocked(o1, o1 + (predObs * 0.25f)))
                o2 = o1 + (predObs * 0.25f);
        }

        // o3 = vertical peek-predicted eye (jump-peek / drop-off); A5 = vertical-only fallback if the diagonal is
        // blocked. Additive origin (only fires when it clears from o1).
        var useO3 = false;
        var o3    = o1;
        if (lookaheadSeconds > 0f && MathF.Abs(rs.Velocity.Z) > MinPredSpeed)
        {
            var zVel     = MathF.Max(-maxPredSpeed, MathF.Min(maxPredSpeed, rs.Velocity.Z));
            var vertical = new Vector3(0f, 0f, zVel * lookaheadSeconds);
            o3 = o1 + predObs + vertical;
            if (!los.IsBlocked(o1, o3))
            {
                useO3 = true;
            }
            else
            {
                o3    = o1 + vertical;
                useO3 = !los.IsBlocked(o1, o3);
            }
        }

        // o4 = un-crouch eye (A7): a ducked observer's velocity ≈ 0, so raise the eye to standing to predict the
        // sightline the un-crouch opens. Dropped if peeking from inside geometry.
        var useO4 = false;
        var o4    = o1;
        if (rs.Ducked)
        {
            o4    = o1 + new Vector3(0f, 0f, UnCrouchEyeRise);
            useO4 = !los.IsBlocked(o1, o4);
        }

        // o5..o8 = RTT-scaled shoulder peeks. o5/o6 flank the real eye, o7/o8 the predicted eye; SafeOrigin reverts a
        // shoulder that would peek from inside geometry back to its base. Skipped entirely when disabled (base 0).
        var shoulderOff = ShoulderOffset(rs.Rtt, in tuning);
        var o5 = o1;
        var o6 = o1;
        var o7 = o2;
        var o8 = o2;
        if (shoulderOff > 0f)
        {
            var shoulder = EyeRight(rs.Yaw) * shoulderOff;
            o5 = SafeOrigin(los, o1, o1 - shoulder);
            o6 = SafeOrigin(los, o1, o1 + shoulder);
            o7 = SafeOrigin(los, o2, o2 - shoulder);
            o8 = SafeOrigin(los, o2, o2 + shoulder);
        }

        // O2IsO1 uses EXACT equality (o2 was assigned `= o1` when it collapsed) — it gates the ray-dedup below, and
        // removing only bitwise-duplicate rays cannot change the OR-of-clears, so the boolean is preserved.
        return new ReceiverGeom(
            lookaheadSeconds, peekMargin, maxPredSpeed, o1, o2, o2 == o1, useO3, o3, useO4, o4, o5, o6, o7, o8);
    }

    /// <summary>
    ///     True if every one of the sampled rays from the receiver to the (possibly peek-predicted) sender AABB is
    ///     occluded by static world geometry — i.e. the pair is LOS-blocked. Any single clear ray returns false
    ///     (visible) immediately.
    ///
    ///     <para><b>Fail-open / monotone.</b> The ray set is built by first laying down o1's full now-box baseline
    ///     and then ONLY appending: o2 / o3 / o4 peek rays and the A3 / A6 predicted-corner rays. Nothing is ever
    ///     removed or relocated, so <c>blocked == !(∃ clear ray)</c> can only ever flip hidden→visible as more rays
    ///     are added — never the reverse. The <paramref name="lastClear" /> temporal cache only reorders the
    ///     sweep, which cannot change an OR — same boolean.</para>
    /// </summary>
    private static bool PairBlocked(
        in ReceiverGeom  geom,
        in SampledPlayer ss,
        LosQuery         los,
        SmokeSnapshot?   smoke,
        float            ageAdvance,
        byte[]?          lastClear,
        int              pairIdx)
    {
        // Receiver-side origins (o1..o8) + effective lookahead are precomputed ONCE per receiver row in
        // BuildReceiverGeom and reused for every sender — recomputing them per pair (incl. ~10 BVH setup queries)
        // was pure waste. Sender-side prediction below still uses the receiver's effective lookahead / peek tuning.
        var o1               = geom.O1;
        var o2               = geom.O2;
        var lookaheadSeconds = geom.Lookahead;
        var peekMargin       = geom.PeekMargin;
        var maxPredSpeed     = geom.MaxPredSpeed;

        // ── Target AABB (NOW box) — the box is NEVER relocated; the merge only ADDS points (A3) ──────────────
        // ±X/±Y inflated by BoundsInflate, but the box FLOOR is clamped to origin.Z + FloorEpsilon (never pushed
        // under the ground).
        var nowMin = new Vector3(
            ss.Origin.X + ss.Mins.X - BoundsInflate,
            ss.Origin.Y + ss.Mins.Y - BoundsInflate,
            ss.Origin.Z + FloorEpsilon);
        var nowMax = ss.Origin + ss.Maxs + new Vector3(BoundsInflate, BoundsInflate, BoundsInflate);

        Span<Vector3> staticPts = stackalloc Vector3[StaticPoints];
        BuildTargetPoints(nowMin, nowMax, staticPts);

        // A3 (MONOTONICITY FIX): when the target strafes and its box travels UNOBSTRUCTED, ADD the future box's
        // leading corners as EXTRA points (sampled from o1) — NEVER relocate the now-box points. Relocating the
        // sampled box onto the union of now+future (the old behaviour) could move a currently-clear ray off a
        // fast-strafing VISIBLE target and flip it HIDDEN. The now-box points above are always sampled; these
        // merely extend coverage to where the target is heading.
        Span<Vector3> futureCorners = stackalloc Vector3[A3FutureCorners];
        var useFutureCorners = false;
        var predTgt = PredictionOffset(ss.Velocity, lookaheadSeconds, peekMargin, maxPredSpeed);
        if (predTgt.X != 0f || predTgt.Y != 0f)
        {
            var centerNow = (nowMin + nowMax) * 0.5f;
            if (!los.IsBlocked(centerNow, centerNow + predTgt)) // only if the box actually travels unobstructed
            {
                BuildBoxCorners(nowMin + predTgt, nowMax + predTgt, futureCorners);
                useFutureCorners = true;
            }
        }

        // A6: jumping / falling TARGET prediction — the sender-side mirror of the o3 (vertical observer) fix.
        // When the target has real vertical speed, ADD 4 predicted corners (top corners lifted by vz·s while
        // rising, bottom corners dropped by vz·s while falling) as EXTRA points sampled from o1 AND o2. Additive:
        // extra target points can only add clear-ray chances.
        Span<Vector3> vertCorners = stackalloc Vector3[A6VerticalCorners];
        var useVertCorners = false;
        if (lookaheadSeconds > 0f && MathF.Abs(ss.Velocity.Z) > MinPredSpeed)
        {
            var vz = MathF.Max(-maxPredSpeed, MathF.Min(maxPredSpeed, ss.Velocity.Z));
            var dz = vz * lookaheadSeconds;
            var zc = vz > 0f ? nowMax.Z + dz : nowMin.Z + dz; // rising → above top; falling → below bottom
            BuildXyCorners(nowMin, nowMax, zc, vertCorners);
            useVertCorners = true;
        }

        // Muzzle point (upstream held-weapon muzzle sampling): the barrel of the target's active gun juts out past
        // the body, so an observer who can see the muzzle can see the player. World point = origin + forward·length
        // at a crouch-scaled MuzzleZ. Only guns produce a length (knife/grenade/none → 0 → skipped). Additive.
        var muzzleLen  = MuzzleLength(ss.WeaponItemDef);
        var haveMuzzle = muzzleLen > 0f;
        var muzzlePt   = default(Vector3);
        if (haveMuzzle)
        {
            var fwd = EyeForward(ss.Yaw);
            var mz  = ss.Origin.Z + AdjustedLocalZ(ss.Mins.Z, ss.Maxs.Z, MuzzleZ);
            muzzlePt = new Vector3((ss.Origin.X + (fwd.X * muzzleLen)), (ss.Origin.Y + (fwd.Y * muzzleLen)), mz);
        }

        // ── Assemble the ADDITIVE ray set (origins × target points) ──────────────────────────────────────────
        Span<Vector3> rayO = stackalloc Vector3[MaxRays];
        Span<Vector3> rayT = stackalloc Vector3[MaxRays];
        var n = 0;

        // (1) o1 real eye → ALL now-box points — the immovable baseline (A3 keeps these on the now box).
        for (var i = 0; i < StaticPoints; i++)
        {
            rayO[n] = o1;
            rayT[n] = staticPts[i];
            n++;
        }

        // (2) o2 horizontal-peek eye → 8 shared + 4 mid-edge points (A1). SKIPPED when o2 exactly collapsed onto o1
        // (stationary / fully-blocked peek): those 12 points are already sampled from o1 in group (1), so the o2
        // rays would be bitwise duplicates — dropping duplicates can't change the OR-of-clears (fail-open holds).
        if (!geom.O2IsO1)
        {
            for (var i = 0; i < SharedPoints; i++)
            {
                rayO[n] = o2;
                rayT[n] = staticPts[i];
                n++;
            }
            for (var i = 0; i < MidEdgePoints; i++)
            {
                rayO[n] = o2;
                rayT[n] = staticPts[MidEdgeStart + i];
                n++;
            }
        }

        // (3) o3 vertical-peek eye → 8 shared points (A8: was 5), when used.
        if (geom.UseO3)
        {
            for (var i = 0; i < SharedPoints; i++)
            {
                rayO[n] = geom.O3;
                rayT[n] = staticPts[i];
                n++;
            }
        }

        // (4) o4 un-crouch eye → 8 shared points (A7), when used.
        if (geom.UseO4)
        {
            for (var i = 0; i < SharedPoints; i++)
            {
                rayO[n] = geom.O4;
                rayT[n] = staticPts[i];
                n++;
            }
        }

        // (4b) o5..o8 RTT-scaled shoulder eyes → 8 shared points each. o5/o6 flank the real eye; o7/o8 flank the
        // predicted eye — but when o2 collapsed onto o1, o7/o8 are bitwise-identical to o5/o6, so skip them. A
        // shoulder that reverted to its base is likewise a duplicate and skipped by AppendSharedRays.
        n = AppendSharedRays(geom.O5, o1, staticPts, rayO, rayT, n);
        n = AppendSharedRays(geom.O6, o1, staticPts, rayO, rayT, n);
        if (!geom.O2IsO1)
        {
            n = AppendSharedRays(geom.O7, o2, staticPts, rayO, rayT, n);
            n = AppendSharedRays(geom.O8, o2, staticPts, rayO, rayT, n);
        }

        // (5) A3 future-box leading corners → o1, when the box travels unobstructed.
        if (useFutureCorners)
        {
            for (var i = 0; i < A3FutureCorners; i++)
            {
                rayO[n] = o1;
                rayT[n] = futureCorners[i];
                n++;
            }
        }

        // (6) A6 vertical-predicted corners → o1 AND o2, when the target is jumping / falling. The o2 copy is
        // skipped when o2 collapsed onto o1 (it would be a bitwise duplicate of the o1 copy).
        if (useVertCorners)
        {
            for (var i = 0; i < A6VerticalCorners; i++)
            {
                rayO[n] = o1;
                rayT[n] = vertCorners[i];
                n++;
            }
            if (!geom.O2IsO1)
            {
                for (var i = 0; i < A6VerticalCorners; i++)
                {
                    rayO[n] = o2;
                    rayT[n] = vertCorners[i];
                    n++;
                }
            }
        }

        // (7) Held-weapon muzzle point → o1 AND o2, when the target holds a gun. Additive (barrel-visible ⇒ reveal).
        // The o2 copy is skipped when o2 collapsed onto o1.
        if (haveMuzzle)
        {
            rayO[n] = o1;
            rayT[n] = muzzlePt;
            n++;
            if (!geom.O2IsO1)
            {
                rayO[n] = o2;
                rayT[n] = muzzlePt;
                n++;
            }
        }

        // ── Sweep: any clear ray ⇒ visible (first-clear early-out). No budget cap. ────────────────────────────
        if (lastClear is not null)
        {
            // Temporal coherence: test the ray that cleared this pair LAST pass first. Reordering an OR-of-clears
            // sweep cannot change its boolean, so this is answer-preserving — it just reaches the early-out sooner
            // when the world barely moved between passes. An out-of-range hint (map change, or a conditional group
            // toggled so the ray count shrank) is skipped by the `hint < n` guard; an in-range hint that no longer
            // clears is simply tested once and folded into the full sweep below — either way the boolean is exact.
            int hint = lastClear[pairIdx];
            if (hint < n && RayClear(los, smoke, ageAdvance, rayO[hint], rayT[hint]))
                return false; // still visible via the remembered ray — keep the hint

            for (var i = 0; i < n; i++)
            {
                if (i == hint)
                    continue; // already tested above
                if (RayClear(los, smoke, ageAdvance, rayO[i], rayT[i]))
                {
                    lastClear[pairIdx] = (byte) i;
                    return false;
                }
            }

            lastClear[pairIdx] = NoClearHint; // fully blocked — no clear ray to remember
            return true;
        }

        // No temporal cache (e.g. ComputeMatrixParallel): plain in-order sweep. Boolean-identical.
        for (var i = 0; i < n; i++)
        {
            if (RayClear(los, smoke, ageAdvance, rayO[i], rayT[i]))
                return false;
        }

        return true;
    }

    /// <summary>
    ///     A ray is CLEAR when the static world does not block it AND the captured smoke (if any) does not block
    ///     it either. This is the smoke-aware per-ray test — see <see cref="SmokeOcclusion.LineBlocked" />. The
    ///     OR-of-clears sweep is unchanged: a pair is blocked only when EVERY ray fails this test.
    ///
    ///     <para><b>⚠️ Smoke makes this test the ONE non-fail-open path.</b> A smoke false-positive here hides an
    ///     otherwise-visible player, so the capture side must feed <paramref name="smoke" /> only fully-validated,
    ///     non-torn grids (it drops ALL smoke on any read failure). When <paramref name="smoke" /> is null the test
    ///     collapses to the plain world-LOS check.</para>
    /// </summary>
    private static bool RayClear(LosQuery los, SmokeSnapshot? smoke, float ageAdvance, Vector3 o, Vector3 t)
        => !los.IsBlocked(o, t) && (smoke is null || !SmokeOcclusion.LineBlocked(smoke, o, t, ageAdvance, los));

    /// <summary>
    ///     15 sample points on the NOW target AABB: 4 top corners, center, upper, all 4 bottom corners (both
    ///     diagonals), the 4 mid-height edge columns (A1) and a low-center 25%-height point (A2). See the index
    ///     layout on <see cref="StaticPoints" />. Built ONLY on the now box — the target-prediction merge adds
    ///     extra points (A3) rather than relocating these.
    /// </summary>
    private static void BuildTargetPoints(Vector3 bmin, Vector3 bmax, Span<Vector3> pts)
    {
        var cx     = (bmin.X + bmax.X) * 0.5f;
        var cy     = (bmin.Y + bmax.Y) * 0.5f;
        var midZ   = (bmin.Z + bmax.Z) * 0.5f;
        var height = bmax.Z - bmin.Z;

        pts[0] = new Vector3(bmin.X, bmin.Y, bmax.Z); // 4 top corners
        pts[1] = new Vector3(bmax.X, bmin.Y, bmax.Z);
        pts[2] = new Vector3(bmin.X, bmax.Y, bmax.Z);
        pts[3] = new Vector3(bmax.X, bmax.Y, bmax.Z);
        pts[4] = new Vector3(cx, cy, midZ);                      // center
        pts[5] = new Vector3(cx, cy, bmin.Z + (0.75f * height)); // "upper"
        pts[6] = new Vector3(bmin.X, bmin.Y, bmin.Z);            // bottom corners — first diagonal
        pts[7] = new Vector3(bmax.X, bmax.Y, bmin.Z);
        pts[8] = new Vector3(bmax.X, bmin.Y, bmin.Z);            // bottom corners — other diagonal
        pts[9] = new Vector3(bmin.X, bmax.Y, bmin.Z);

        // A1: 4 mid-height edge columns — a half-exposed target at a window/corner edge used to fall between all
        // points (top corners too high, bottom corners too low, center column dead centre) → wrongly hidden.
        pts[10] = new Vector3(bmin.X, bmin.Y, midZ);
        pts[11] = new Vector3(bmax.X, bmin.Y, midZ);
        pts[12] = new Vector3(bmin.X, bmax.Y, midZ);
        pts[13] = new Vector3(bmax.X, bmax.Y, midZ);

        // A2: low-center 25% height — knee/waist-slit visibility; the center column otherwise voided 0–50%.
        pts[14] = new Vector3(cx, cy, bmin.Z + (0.25f * height));
    }

    /// <summary>8 corners (4 top at <c>mx.Z</c>, 4 bottom at <c>mn.Z</c>) of the box [<paramref name="mn" />,<paramref name="mx" />].</summary>
    private static void BuildBoxCorners(Vector3 mn, Vector3 mx, Span<Vector3> c)
    {
        c[0] = new Vector3(mn.X, mn.Y, mx.Z);
        c[1] = new Vector3(mx.X, mn.Y, mx.Z);
        c[2] = new Vector3(mn.X, mx.Y, mx.Z);
        c[3] = new Vector3(mx.X, mx.Y, mx.Z);
        c[4] = new Vector3(mn.X, mn.Y, mn.Z);
        c[5] = new Vector3(mx.X, mn.Y, mn.Z);
        c[6] = new Vector3(mn.X, mx.Y, mn.Z);
        c[7] = new Vector3(mx.X, mx.Y, mn.Z);
    }

    /// <summary>4 XY corners of [<paramref name="mn" />,<paramref name="mx" />] at a single height <paramref name="z" />.</summary>
    private static void BuildXyCorners(Vector3 mn, Vector3 mx, float z, Span<Vector3> c)
    {
        c[0] = new Vector3(mn.X, mn.Y, z);
        c[1] = new Vector3(mx.X, mn.Y, z);
        c[2] = new Vector3(mn.X, mx.Y, z);
        c[3] = new Vector3(mx.X, mx.Y, z);
    }

    /// <summary>
    ///     upstream visibility_prediction_offset: 2D velocity scaled to the predicted travel distance (>= peek
    ///     margin), Z zeroed. Returns zero for slow / stopped movers.
    /// </summary>
    private static Vector3 PredictionOffset(Vector3 vel, float s, float peekMargin, float maxPredSpeed)
    {
        var speed = MathF.Sqrt((vel.X * vel.X) + (vel.Y * vel.Y));
        if (s <= 0f || speed <= MinPredSpeed) // k_min_prediction_speed = 1
            return Vector3.Zero;

        var capped = MathF.Min(speed, maxPredSpeed); // k_max_prediction_speed
        var dist   = MathF.Max(capped * s, peekMargin);
        var scale  = dist / speed;
        return new Vector3(vel.X * scale, vel.Y * scale, 0f);
    }

    /// <summary>The eye's right vector (unit, XY plane) from the observer yaw — upstream <c>eye_right</c>.</summary>
    private static Vector3 EyeRight(float yawDegrees)
    {
        var yaw = yawDegrees * DegToRad;
        return new Vector3(MathF.Sin(yaw), -MathF.Cos(yaw), 0f);
    }

    /// <summary>
    ///     upstream <c>safe_origin</c>: never peek from inside geometry. If the shoulder candidate is blocked from
    ///     its base eye (or coincident with it), fall back to the base eye. Keeps every shoulder origin additive.
    /// </summary>
    private static Vector3 SafeOrigin(LosQuery los, Vector3 baseEye, Vector3 candidate)
        => los.IsBlocked(baseEye, candidate) ? baseEye : candidate;

    /// <summary>Two points within upstream <c>k_same_point_epsilon_sq</c> (used to skip a shoulder that is a duplicate).</summary>
    private static bool Same(Vector3 a, Vector3 b)
    {
        var dx = a.X - b.X;
        var dy = a.Y - b.Y;
        var dz = a.Z - b.Z;
        return ((dx * dx) + (dy * dy) + (dz * dz)) <= SamePointEpsSq;
    }

    /// <summary>
    ///     Append the 8 shared target-point rays from <paramref name="origin" /> — unless it collapsed onto
    ///     <paramref name="baseEye" /> (a pure duplicate of an origin already in the sweep). Returns the new count.
    /// </summary>
    private static int AppendSharedRays(
        Vector3 origin, Vector3 baseEye, ReadOnlySpan<Vector3> staticPts, Span<Vector3> rayO, Span<Vector3> rayT, int n)
    {
        if (Same(origin, baseEye))
            return n;
        for (var i = 0; i < SharedPoints; i++)
        {
            rayO[n] = origin;
            rayT[n] = staticPts[i];
            n++;
        }
        return n;
    }

    /// <summary>upstream <c>eye_forward</c>: the forward vector (unit, XY plane) from a yaw — aims the muzzle point.</summary>
    private static Vector3 EyeForward(float yawDegrees)
    {
        var yaw = yawDegrees * DegToRad;
        return new Vector3(MathF.Cos(yaw), MathF.Sin(yaw), 0f);
    }

    /// <summary>
    ///     upstream <c>adjusted_local_z</c>: map a standing-model local Z onto the target's live (possibly crouched)
    ///     hull so the muzzle point drops when the target ducks. Below the pelvis the mapping is 1:1; above it scales
    ///     by the live upper-body height. Returns a Z offset relative to the target origin (already includes minZ).
    /// </summary>
    private static float AdjustedLocalZ(float minZ, float maxZ, float localZ)
    {
        var height = MathF.Max(1f, maxZ - minZ);
        if (localZ < PelvisHeight)
            return minZ + localZ;
        var standingUpper = StandingPlayerHeight - PelvisHeight;
        var liveUpper     = MathF.Max(0f, height - PelvisHeight);
        return minZ + PelvisHeight + ((localZ - PelvisHeight) * liveUpper / standingUpper);
    }

    /// <summary>
    ///     Barrel length (units) of the target's active weapon by item-definition index — upstream muzzle table:
    ///     pistol 18 / smg 28 / rifle 36 (incl. shotguns/LMGs) / sniper 52. Knife, grenade, or unknown ⇒ 0 (no
    ///     muzzle point). Item-def numbers match CS2FOW's <c>weapon_muzzle_class_from_item_definition</c>.
    /// </summary>
    private static float MuzzleLength(ushort itemDef)
        => itemDef switch
        {
            1 or 2 or 3 or 4 or 30 or 32 or 36 or 61 or 63 or 64                     => 18f, // pistols
            17 or 19 or 23 or 24 or 26 or 33 or 34                                   => 28f, // smgs
            9 or 11 or 38 or 40                                                      => 52f, // snipers
            7 or 8 or 10 or 13 or 14 or 16 or 25 or 27 or 28 or 29 or 35 or 39 or 60 => 36f, // rifles/shotguns/lmg
            _                                                                       => 0f,
        };

    /// <summary>
    ///     Pair-eligibility predicate on raw team ints (2=T, 3=CT). Both sides MUST be a real playing team (T or
    ///     CT — never <c>UnAssigned</c> / <c>Spectator</c>). With <paramref name="filterTeammates" /> off (default)
    ///     only strictly-opposing T-vs-CT pairs are sampled; with it on, same-team pairs (T-vs-T, CT-vs-CT) are
    ///     sampled too.
    ///
    ///     <para><b>Must stay byte-identical to <c>FogOfWarModule.ShouldCull</c>.</b> The worker (this predicate)
    ///     and the game-thread apply (that one) decide pair eligibility INDEPENDENTLY; if they ever disagree, pairs
    ///     strobe (one hides, the other releases every frame).</para>
    /// </summary>
    private static bool ShouldCull(int a, int b, bool filterTeammates)
    {
        var aTeam = a == TeamT || a == TeamCT;
        var bTeam = b == TeamT || b == TeamCT;
        if (!aTeam || !bTeam)
            return false;                // never Spectator / None / UnAssigned

        return filterTeammates || a != b; // teammate mode: any T/CT pair; else strictly opposing
    }
}
