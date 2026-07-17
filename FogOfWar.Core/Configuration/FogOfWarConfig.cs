using System;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;
using Microsoft.Extensions.Logging;

namespace FogOfWar.Configuration;

/// <summary>
///     Fog-of-War anti-wallhack DENIAL config (served by <c>FogOfWarModule</c>). Concept ported from
///     CS2FOW by karola3vax (MIT). FOW is <b>NOT</b> a detector: it does not detect, warn, kick or ban — it
///     stops the server from networking an enemy pawn to a client that provably cannot see it, so a
///     wallhack/ESP has no pawn to read. It is a standalone, <b>opt-in</b> feature toggled purely by
///     <see cref="Enabled" /> (default <c>false</c> → completely inert).
///
///     <para>The oracle bakes the map's static world collision into an immutable BVH at map load (off the game
///     thread, CRC-cached) and recomputes the full enemy visibility matrix on a background worker thread every
///     <see cref="UpdateIntervalMs" />, using the baked BVH (no engine trace, no per-tick budget). The game
///     thread only reads the latest worker snapshot and applies transmit denial, under a fail-open staleness
///     rule: <b>any degradation always errs toward VISIBLE</b> (worker not ready / stalled, bake not loaded,
///     pair missing), so a FOW bug can only ever fail to hide a hidden enemy — it can never hide a genuinely
///     visible one. Weapons a hidden pawn carries are hidden with it via the native owner mechanism.</para>
/// </summary>
public sealed class FogOfWarConfig
{
    /// <summary>
    ///     Master switch. Default <c>false</c> — the module is completely inert (no hooks, no traces, no
    ///     frame hook) until explicitly enabled. OPT-IN.
    /// </summary>
    [JsonPropertyName("enabled")] public bool Enabled { get; set; } = false;

    /// <summary>
    ///     Background-worker recompute cadence in milliseconds: the worker recomputes the FULL enemy visibility
    ///     matrix (every alive-enemy pair, full ray set) against the baked BVH at most this often, then pauses
    ///     until the next tick. Also feeds the observer/target lookahead prediction window. Lower = fresher
    ///     hides at more worker CPU; the work is entirely off the game thread. Default 15 (the 120ms
    ///     min-lookahead clamp keeps the prediction window unchanged, so this just cuts mean pipeline latency;
    ///     staleVisibleMs auto-clamps up at Install if it would fall below two intervals).
    /// </summary>
    [JsonPropertyName("updateIntervalMs")] public int UpdateIntervalMs { get; set; } = 15;

    /// <summary>
    ///     Off-thread visibility-matrix parallelism: the max worker threads the background compute may fan across at
    ///     high player counts (≥ ~12 players). <c>1</c> = serial (default — no extra threads). The matrix is O(pairs),
    ///     so on a busy 32–64 player server raising this (e.g. 3–4) cuts matrix latency by computing receiver rows in
    ///     parallel — each row is an independent, disjoint slice, so the temporal-coherence cache and the boolean
    ///     result are unchanged. Clamped to the box's CPU count. Keep it MODEST on a co-tenant host (many servers per
    ///     box): a 64-player burst across every core would starve the game thread / neighbours. ONLY the off-thread
    ///     compute is affected — the game thread is never parallelised, so its per-tick cost is unchanged.
    /// </summary>
    [JsonPropertyName("maxComputeThreads")] public int MaxComputeThreads { get; set; } = 1;

    /// <summary>
    ///     BASE peek-prediction lookahead (ms) — the window a 0-ping observer gets. Covers the off-thread worker's
    ///     recompute cadence + snapshot age. The effective per-client window is <c>base + ping·rttLookaheadScale</c>,
    ///     capped at <see cref="MaxLookaheadMs" />. Default 180.
    /// </summary>
    [JsonPropertyName("minLookaheadMs")] public int MinLookaheadMs { get; set; } = 180;

    /// <summary>
    ///     Upper clamp (ms) on the per-client lookahead window (base + RTT term). Default 375 (upstream cap). Set to
    ///     0 to disable peek prediction entirely (falls back to o1 baseline + shoulder origins only).
    /// </summary>
    [JsonPropertyName("maxLookaheadMs")] public int MaxLookaheadMs { get; set; } = 375;

    /// <summary>
    ///     Extra lookahead per second of the observer's real RTT (from <c>m_iPing</c>): effective lookahead =
    ///     clamp(<see cref="MinLookaheadMs" /> + pingMs·scale, 0, <see cref="MaxLookaheadMs" />). Upstream
    ///     <c>rtt_lookahead_scale</c> = 1.5 (a 100ms peeker gets +150ms lead). 0 disables RTT scaling (flat base).
    /// </summary>
    [JsonPropertyName("rttLookaheadScale")] public float RttLookaheadScale { get; set; } = 1.5f;

    /// <summary>
    ///     Fallback RTT (ms) used only while a client's <c>m_iPing</c> is still 0 (the brief mid-connect window);
    ///     once real ping populates it is used instead. Default 60. (Formerly the fixed per-client RTT stand-in —
    ///     real per-client ping now drives the lookahead + shoulder scaling.)
    /// </summary>
    [JsonPropertyName("assumedRttMs")] public int AssumedRttMs { get; set; } = 60;

    /// <summary>
    ///     Lateral shoulder-peek origin offset (units) at 0 ping — every observer samples a static ±this shoulder
    ///     peek so a corner-hugging enemy is revealed. Widens with RTT up to <see cref="MaxShoulderUnits" />.
    ///     Upstream <c>shoulder_base_units</c> = 24. 0 disables shoulder origins entirely.
    /// </summary>
    [JsonPropertyName("shoulderBaseUnits")] public float ShoulderBaseUnits { get; set; } = 24.0f;

    /// <summary>
    ///     Shoulder-peek offset added per millisecond of the observer's RTT: offset =
    ///     clamp(pingMs·scale, <see cref="ShoulderBaseUnits" />, <see cref="MaxShoulderUnits" />). Upstream
    ///     <c>shoulder_rtt_scale</c> = 0.64 (a 100ms peeker gets a 64u shoulder). 0 pins the offset at the base.
    /// </summary>
    [JsonPropertyName("shoulderRttScale")] public float ShoulderRttScale { get; set; } = 0.64f;

    /// <summary>Upper clamp (units) on the RTT-scaled shoulder-peek offset. Upstream <c>max_shoulder_units</c> = 128.</summary>
    [JsonPropertyName("maxShoulderUnits")] public float MaxShoulderUnits { get; set; } = 128.0f;

    /// <summary>
    ///     Minimum peek margin (units) added to the predicted travel distance so a corner-peeker is revealed
    ///     slightly before the model strictly requires it (errs visible). Default 21.
    /// </summary>
    [JsonPropertyName("peekMarginUnits")] public float PeekMarginUnits { get; set; } = 21.0f;

    /// <summary>
    ///     Hysteresis hold (ms): once a pair is seen visible it stays visible for at least this long even if
    ///     the next trace says blocked, killing flicker at edges. Default 150.
    /// </summary>
    [JsonPropertyName("visibilityHoldMs")] public int VisibilityHoldMs { get; set; } = 150;

    /// <summary>
    ///     Prediction speed cap (units/s, upstream <c>k_max_prediction_speed</c>): velocity used for the
    ///     lookahead offset is capped at this so a boosted/exploited speed can't push the predicted box past
    ///     the real reveal point. Default 500.
    /// </summary>
    [JsonPropertyName("maxPredictionSpeed")] public float MaxPredictionSpeed { get; set; } = 500.0f;

    /// <summary>
    ///     SAFETY / fail-open invariant: if the newest worker visibility snapshot is older than this many
    ///     milliseconds (a stalled / crashed / not-yet-started worker, or a map whose bake has not finished),
    ///     EVERY pair is FORCED VISIBLE until a fresh snapshot lands. Guarantees a stalled worker can only
    ///     under-hide, never false-hide a visible enemy. Default 250.
    /// </summary>
    [JsonPropertyName("staleVisibleMs")] public int StaleVisibleMs { get; set; } = 250;

    /// <summary>
    ///     Also hide weapons a hidden pawn carries (they leak position to ESP). Uses the native transmit
    ///     owner mechanism (a hooked weapon whose owner controller is hidden from a receiver is cleared with
    ///     it — zero per-pair cost). Default true.
    /// </summary>
    [JsonPropertyName("hideWeapons")] public bool HideWeapons { get; set; } = true;

    /// <summary>
    ///     Dedicated <see cref="Sharp.Shared.Managers.ITransmitManager" /> channel (0..5) FOW claims so it
    ///     composes (AND) with other transmit plugins instead of stomping them. Default 3.
    /// </summary>
    [JsonPropertyName("transmitChannel")] public int TransmitChannel { get; set; } = 3;

    /// <summary>Emit a rolling stats line (µs/trace, ms/tick, pair ages, hidden/visible counts) every 60s. Default false.</summary>
    [JsonPropertyName("debugStatus")] public bool DebugStatus { get; set; } = false;

    /// <summary>
    ///     Extra surface-property names to treat as SEE-THROUGH (non-occluding), unioned with the built-in
    ///     see-through set (chain-link / grates / glass). Case-insensitive, exact match on the resolved
    ///     surface-property name. Use this to plug a custom map's translucent surfaceprop that bakes as an
    ///     occluder (a false-hide) — the bake log prints the top included surface-property names so a live
    ///     false-hide is diagnosable and the offending name can be added here. Empty by default.
    /// </summary>
    [JsonPropertyName("extraSeeThroughProps")] public string[] ExtraSeeThroughProps { get; set; } = Array.Empty<string>();

    /// <summary>
    ///     Also cull SAME-TEAM pairs (T-vs-T, CT-vs-CT), not just enemies. Default <c>false</c>. The "both sides
    ///     are a real playing team (never Spectator/None)" guard still applies. NOTE: the LOS worker and the
    ///     game-thread apply widen this predicate INDEPENDENTLY and must agree — both read this one flag.
    /// </summary>
    [JsonPropertyName("filterTeammates")] public bool FilterTeammates { get; set; } = false;

    /// <summary>
    ///     Live CS2 smoke clouds occlude visibility (a captured smoke that provably blocks the sightline hides the
    ///     enemy behind it). Default <c>false</c> — OPT-IN-WITHIN-OPT-IN. Smoke raw-reads CS2 process memory via
    ///     fixed <see cref="SmokeOffsets" /> every frame on the game thread and is the <b>ONLY non-fail-open
    ///     feature</b> (it can hide an otherwise-visible player); a CS2 update that shifts the smoke layout can turn
    ///     a stale offset into a false-hide or a game-thread crash. An operator must therefore explicitly enable this
    ///     per CS2 build AFTER re-verifying the offsets, and should pin <see cref="SmokeBinarySizeBytes" /> /
    ///     <see cref="SmokeBinaryCrc32" /> so a later build change auto-disables smoke instead of reading stale
    ///     offsets. Any capture/read failure still drops ALL smoke for that pass (fail-open guard).
    /// </summary>
    [JsonPropertyName("smokeOcclusion")] public bool SmokeOcclusion { get; set; } = false;

    /// <summary>
    ///     Throttle: re-copy the (expensive) 32,768-cell smoke density grid at most once every this many game
    ///     frames while smoke is active; in-between frames reuse the last captured grid (the worker ages it forward
    ///     so timing stays correct). Smoke evolves far slower than the 64Hz tick, so a small value cuts the
    ///     per-frame game-thread copy cost with no visible change. Clamped to &gt;= 1 (1 = every frame). Default 3.
    /// </summary>
    [JsonPropertyName("smokeCaptureIntervalFrames")] public int SmokeCaptureIntervalFrames { get; set; } = 3;

    /// <summary>
    ///     Binary gate (paired with <see cref="SmokeBinaryCrc32" />): the expected byte size of the CS2 server
    ///     binary (<c>libserver.so</c>) that the raw <see cref="SmokeOffsets" /> were verified against. When SET
    ///     (&gt; 0) and the loaded binary's size does not match, smoke occlusion is DISABLED at Install (a CS2
    ///     update shifted the layout → stale offsets would risk a false-hide / crash). Default 0 = unset (smoke is
    ///     allowed but logs a loud "offsets unverified" warning). Pin this after re-verifying offsets for a build.
    /// </summary>
    [JsonPropertyName("smokeBinarySizeBytes")] public long SmokeBinarySizeBytes { get; set; } = 0;

    /// <summary>
    ///     Binary gate (paired with <see cref="SmokeBinarySizeBytes" />): the expected CRC-32 (hex, e.g.
    ///     <c>"a1b2c3d4"</c>) of <c>libserver.so</c> the raw <see cref="SmokeOffsets" /> were verified against. When
    ///     SET and the loaded binary's CRC does not match, smoke occlusion is DISABLED at Install. Hashed ONCE at
    ///     Install (never per frame). Default empty = unset. Optional refinement over the cheaper size check.
    /// </summary>
    [JsonPropertyName("smokeBinaryCrc32")] public string SmokeBinaryCrc32 { get; set; } = string.Empty;

    /// <summary>
    ///     Radius (units) around an HE detonation within which the blast punches a temporary viewing channel
    ///     through smoke. Default 100. Set &lt;= 0 to disable HE clearing.
    /// </summary>
    [JsonPropertyName("heClearRadiusUnits")] public float HeClearRadiusUnits { get; set; } = 100.0f;

    /// <summary>
    ///     How long (seconds) an HE detonation keeps its smoke channel open. Default 2.5. Set &lt;= 0 to disable.
    /// </summary>
    [JsonPropertyName("heClearSeconds")] public float HeClearSeconds { get; set; } = 2.5f;

    /// <summary>
    ///     Raw engine memory offsets used to read a smoke projectile's density grid (from CS2FOW's
    ///     <c>cs2fow.games.txt</c>, verified against a specific CS2 build). Exposed in config so a CS2 update that
    ///     shifts these can be hotfixed WITHOUT a rebuild. Validated at Install; if invalid, smoke occlusion is
    ///     disabled (fail-open — the rest of FOW is unaffected).
    /// </summary>
    [JsonPropertyName("smokeOffsets")] public SmokeOffsets SmokeOffsets { get; set; } = new();

    private static readonly JsonSerializerOptions JsonOpts = new()
    {
        PropertyNameCaseInsensitive = true,
        ReadCommentHandling         = JsonCommentHandling.Skip,
        AllowTrailingCommas         = true,
        WriteIndented               = true,
        DefaultIgnoreCondition      = JsonIgnoreCondition.WhenWritingNull,
    };

    /// <summary>
    ///     Load <c>configs/fogofwar.json</c> (write defaults if missing). Tolerates comments / trailing commas,
    ///     never throws — a bad file falls back to defaults (module stays inert since <see cref="Enabled" />
    ///     defaults false).
    /// </summary>
    public static FogOfWarConfig Load(string sharpPath, ILogger logger)
    {
        var path = Path.Combine(sharpPath, "configs", "fogofwar.json");
        try
        {
            if (!File.Exists(path))
            {
                var def = new FogOfWarConfig();
                Directory.CreateDirectory(Path.GetDirectoryName(path)!);
                File.WriteAllText(path, JsonSerializer.Serialize(def, JsonOpts));
                logger.LogInformation("[FogOfWar] Wrote default config to {Path}", path);
                return def;
            }

            var cfg = JsonSerializer.Deserialize<FogOfWarConfig>(File.ReadAllText(path), JsonOpts);
            if (cfg is null)
            {
                logger.LogError("[FogOfWar] fogofwar.json deserialized to null — using defaults");
                return new FogOfWarConfig();
            }
            return cfg;
        }
        catch (Exception e)
        {
            logger.LogError(e, "[FogOfWar] Failed to load fogofwar.json — using defaults");
            return new FogOfWarConfig();
        }
    }
}

/// <summary>
///     Raw engine offsets for reading a <c>smokegrenade_projectile</c>'s smoke volume + density storage. Defaults
///     are CS2FOW's Linux values (<c>cs2fow.games.txt</c>, verified against CS2 build 24209309). All are relative:
///     <see cref="Volume" /> is added to the entity pointer; <see cref="Center" /> / <see cref="StartTime" /> /
///     <see cref="Storage" /> / <see cref="Frame" /> are read from the volume; the <c>Storage*</c> offsets index
///     into the storage block the <see cref="Storage" /> pointer points at.
/// </summary>
public sealed class SmokeOffsets
{
    /// <summary>Entity pointer → smoke volume (smoke_volume_offset_linux).</summary>
    [JsonPropertyName("volume")] public int Volume { get; set; } = 3504;

    /// <summary>Volume → cloud center Vector (smoke_center_offset).</summary>
    [JsonPropertyName("center")] public int Center { get; set; } = 212;

    /// <summary>Volume → start-time float (smoke_start_time_offset).</summary>
    [JsonPropertyName("startTime")] public int StartTime { get; set; } = 12;

    /// <summary>Volume → storage pointer (smoke_storage_offset).</summary>
    [JsonPropertyName("storage")] public int Storage { get; set; } = 112;

    /// <summary>Volume → active frame index 0/1 (smoke_frame_offset).</summary>
    [JsonPropertyName("frame")] public int Frame { get; set; } = 236;

    /// <summary>Storage → opaque-cell mask (4096 bytes) (k_smoke_storage_mask_offset).</summary>
    [JsonPropertyName("storageMask")] public int StorageMask { get; set; } = 8;

    /// <summary>Storage → density grid base (k_smoke_storage_density_offset = 0x3008).</summary>
    [JsonPropertyName("storageDensity")] public int StorageDensity { get; set; } = 0x3008;

    /// <summary>Per-frame stride within the density storage (k_smoke_storage_frame_stride = 0x80000).</summary>
    [JsonPropertyName("storageFrameStride")] public long StorageFrameStride { get; set; } = 0x80000;

    /// <summary>Per-cell stride within a density frame; the float lives at offset 0 (k_smoke_storage_cell_stride = 0x10).</summary>
    [JsonPropertyName("storageCellStride")] public int StorageCellStride { get; set; } = 0x10;

    /// <summary>
    ///     Sanity-check the offsets before the module raw-reads with them. A garbage offset (e.g. a mangled config
    ///     after a CS2 update) would otherwise walk arbitrary memory. Fields must be non-negative and within a
    ///     generous cap; strides must be positive and the cell stride at least a float wide. Returns false ⇒ the
    ///     module disables smoke occlusion (fail-open) rather than risk a bad read.
    /// </summary>
    public bool Validate()
        => Volume is >= 0 and < 65536
           && Center is >= 0 and < 65536
           && StartTime is >= 0 and < 65536
           && Storage is >= 0 and < 65536
           && Frame is >= 0 and < 65536
           && StorageMask is >= 0 and < 65536
           && StorageDensity is >= 0 and < (1 << 24)
           && StorageFrameStride is > 0 and < (1L << 28)
           && StorageCellStride is >= 4 and < 4096;
}
