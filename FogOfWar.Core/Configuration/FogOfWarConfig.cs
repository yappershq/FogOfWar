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

    /// <summary>Lower clamp (ms) on the peek-prediction lookahead window. Default 180.</summary>
    [JsonPropertyName("minLookaheadMs")] public int MinLookaheadMs { get; set; } = 180;

    /// <summary>Upper clamp (ms) on the peek-prediction lookahead window. Default 250.</summary>
    [JsonPropertyName("maxLookaheadMs")] public int MaxLookaheadMs { get; set; } = 250;

    /// <summary>
    ///     Stand-in for per-client RTT (ModSharp exposes no per-client latency). Added to
    ///     <see cref="UpdateIntervalMs" /> to form the raw lookahead before clamping. Default 60; with the
    ///     default 30ms interval the raw 90ms is lifted to the 180ms <see cref="MinLookaheadMs" /> floor →
    ///     effective default lookahead 180ms. Raise on high-ping populations to widen the reveal margin.
    /// </summary>
    [JsonPropertyName("assumedRttMs")] public int AssumedRttMs { get; set; } = 60;

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
