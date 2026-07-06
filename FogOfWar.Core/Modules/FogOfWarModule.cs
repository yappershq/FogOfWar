// Fog-of-War visibility culling — concept ported from CS2FOW by karola3vax
// https://github.com/karola3vax/CS2FOW  (MIT License, Copyright (c) karola3vax)
// From-scratch C# implementation for ModSharp: the transmit-denial concept, the prediction/sampling
// model (lookahead, peek margin, visibility hold, k_max_prediction_speed=500) and cfg defaults follow
// upstream. Visibility is computed against a BVH baked from the map's static world_physics collision
// (opacity-filtered), recomputed on a background WORKER thread; the game thread only reads the latest
// worker snapshot and applies the transmit denial.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using FogOfWar.Configuration;
using FogOfWar.Engine;
using Microsoft.Extensions.Logging;
using Sharp.Shared.Enums;
using Sharp.Shared.GameEntities;
using Sharp.Shared.Listeners;
using Sharp.Shared.Objects;
using Sharp.Shared.Types;
using Sharp.Shared.Units;
using Vector3 = System.Numerics.Vector3;

namespace FogOfWar;

/// <summary>
///     Fog-of-War anti-wallhack DENIAL module (concept from CS2FOW by karola3vax, MIT). NOT a detector: it
///     never warns / kicks / bans and is deliberately independent of the global <c>detectionMode</c> switch.
///     Instead it stops the server from networking an enemy pawn to a client that provably cannot see it, so
///     a wallhack / ESP has no pawn state (position / animation / health pose) to read.
///
///     <para><b>Denial mechanism (<see cref="Sharp.Shared.Managers.ITransmitManager" />).</b> PlayerPawns
///     cannot be hooked directly (the engine hard-rejects it), so the module hooks each player's CONTROLLER
///     with <c>AddEntityHooks(controller, defaultTransmit:true)</c>. When a hooked sender controller's state
///     for a receiver is set to <c>false</c> on our channel, the engine's CheckTransmit post-pass resolves
///     the controller's pawn and clears the PAWN's transmit bit — the controller itself is never culled, so
///     scoreboard / team data stay intact. State is pushed per pair with
///     <c>SetEntityState(senderCtrlIndex, receiverCtrlIndex, visible, channel)</c> on a dedicated channel
///     (config <c>transmitChannel</c>) so it composes (AND) with other transmit plugins.</para>
///
///     <para><b>Visibility oracle (BVH, off the game thread).</b> On map load the map's static world collision
///     (<c>world_physics</c>, opacity-filtered so fences / grates / glass do not occlude) is baked into an
///     immutable <see cref="LosQuery" /> BVH — asynchronously, CRC-cached (<see cref="BakeCache" />) — without
///     blocking the map change. A background <b>worker thread</b> then continuously (every
///     <c>updateIntervalMs</c>, no budget cap) recomputes the FULL enemy visibility matrix — every alive-enemy
///     pair, the full 2(+1 vertical peek)-observer × 8-target ray set with peek prediction and floor-clamped
///     target box (§2.2) —
///     via <see cref="LosSampler" /> against that immutable BVH, into an immutable snapshot. The game thread
///     only READS the latest snapshot (volatile swap) and applies denial.</para>
///
///     <para><b>Fail-open invariant (anti-cheat correctness).</b> Every degradation errs VISIBLE, never
///     hidden: (1) if the newest worker snapshot is null (worker not started, bake not loaded) or older than
///     <c>staleVisibleMs</c> (worker stalled), EVERY pair is forced visible; (2) any pair missing from the
///     snapshot is visible; (3) the per-pair re-validation gate forces visible for dead / spectating / HLTV /
///     bot receivers, same-team pairs, dead senders, and any missing hook. A FOW bug can only ever fail to
///     hide a hidden enemy — it can never hide a genuinely visible one (a gameplay-breaking false-hide).</para>
///
///     <para><b>Threads.</b> The game thread owns ALL ModSharp state: it snapshots live pawn state into plain
///     structs (<see cref="LosSampler.SampledPlayer" />, System.Numerics only) and hands them to the worker
///     (volatile publish); the worker hands the computed matrix back (volatile publish) and NEVER touches a
///     live entity / manager. <c>SetEntityState</c> / hooks / entity reads happen ONLY on the game thread. The
///     map bake runs on a thread-pool task. The worker thread is stopped + joined in <see cref="Uninstall" />.
///     Only slot (0..63) and <see cref="EntityIndex" /> are stored; controllers / pawns / clients are
///     re-resolved live each frame and never cached across frames.</para>
/// </summary>
internal sealed class FogOfWarModule : IClientListener, IEntityListener, IGameListener
{
    // IClientListener / IEntityListener / IGameListener all use ApiVersion 1 — one property serves all three.
    public int ListenerVersion  => IClientListener.ApiVersion;
    public int ListenerPriority => 0;

    private readonly InterfaceBridge _bridge;
    private readonly FogOfWarConfig _config;
    private readonly ILogger         _logger;

    private const int TickRate = 64; // CS2 fixed server tick.
    private const int Slots    = 64; // max player slots.
    private const int SlowMs   = 250; // full re-push / re-hook / weapon-reconcile cadence.

    // Minimum plausible occluder-triangle count for a real map's world_physics. A winner below this is a bake
    // failure (a stub / placeholder / wrong pick), NOT a real map — treated as fail-open (all visible) so FOW
    // never bakes a near-empty occluder set that would silently under-hide.
    private const int MinReasonableTriangles = 3000;

    // Fail-open warning throttle: a persistently-throwing frame logs at most once per this many ticks.
    private const int ThrowLogCooldownTicks = 5 * TickRate;

    // Same 5s throttle for the OFF-thread worker's compute-fault warning (wall-clock ms; the worker can't read
    // the game TickCount).
    private const long ThrowLogCooldownMs = 5000;

    // ── Cached config (read once at Install so the hot path never re-reads config props) ─────────────────
    private int   _channel;
    private float _lookaheadSeconds; // constant per config (no per-client RTT exposed).
    private float _peekMargin;
    private float _maxPredSpeed;
    private int   _holdTicks;
    private int   _staleTicks;
    private int   _workerIntervalMs; // background worker recompute cadence.
    private int   _slowTicks;
    private bool  _hideWeapons;
    private bool  _debugStatus;

    // ── Per-frame snapshot (one per slot) — captured on the game thread ──────────────────────────────────
    private struct PawnSnap
    {
        public bool        Valid;     // controller connected + pawn alive this frame
        public bool        Human;     // controlling client is a real human (bots excluded as receivers)
        public bool        Ducked;    // observer fully crouched this frame (A7 — enables the un-crouch origin o4)
        public CStrikeTeam Team;
        public Vector      Eye;       // observer origin (receiver eye)
        public Vector      Origin;    // target abs origin (sender)
        public Vector      Velocity;
        public Vector      Mins;      // local collision mins
        public Vector      Maxs;      // local collision maxs
        public EntityIndex CtrlIndex; // controller entity index (== slot + 1)
    }

    private readonly PawnSnap[] _snap = new PawnSnap[Slots];

    // ── Per-pair state, flat [receiverSlot * 64 + senderSlot] — game thread only ─────────────────────────
    private readonly int[]  _revealedUntil = new int[Slots * Slots];  // hysteresis hold expiry (tick)
    private readonly bool[] _pushedHidden  = new bool[Slots * Slots]; // last state pushed to engine (true = hidden)

    // ── Worker hand-off (game thread ↔ worker thread, immutable snapshots + volatile swap) ────────────────
    // Immutable per-frame player state handed game → worker. Generation stamps which map's positions these are:
    // the worker refuses to pair old-map positions with the new map's BVH (would compute a garbage matrix that
    // the game thread could briefly accept at map start → false-hide window).
    private sealed class WorkerInput
    {
        public readonly LosSampler.SampledPlayer[] Players;
        public readonly int                        Max;
        public readonly int                        Tick;
        public readonly int                        Generation;

        public WorkerInput(LosSampler.SampledPlayer[] players, int max, int tick, int generation)
        {
            Players    = players;
            Max        = max;
            Tick       = tick;
            Generation = generation;
        }
    }

    // Immutable computed matrix handed worker → game. Blocked[idx] is true only for an enemy pair the worker
    // computed AND found fully occluded; everything else is false (= visible). InputTick is the game tick of
    // the player state it was computed from — the fail-open staleness reference. Generation stamps which map's
    // BVH produced it, so the game thread can reject a snapshot computed against a now-superseded map (a pass
    // that finished against the PREVIOUS map's geometry right across a map change → could false-hide otherwise).
    private sealed class VisSnapshot
    {
        public readonly bool[] Blocked;
        public readonly int    InputTick;
        public readonly int    Generation;

        public VisSnapshot(bool[] blocked, int inputTick, int generation)
        {
            Blocked    = blocked;
            InputTick  = inputTick;
            Generation = generation;
        }
    }

    // The baked BVH bound to the map generation it was built for — a SINGLE atomic reference so the worker reads
    // geometry + generation together (they can never tear apart across a map change).
    private sealed class LosState
    {
        public readonly LosQuery Query;
        public readonly int      Generation;

        public LosState(LosQuery query, int generation)
        {
            Query      = query;
            Generation = generation;
        }
    }

    private WorkerInput? _input;    // volatile via Volatile.Read/Write
    private VisSnapshot? _vis;      // volatile via Volatile.Read/Write
    private LosState?    _losState; // volatile via Volatile.Read/Write — baked BVH + generation (null until ready)

    // One worker run's private control block. A FRESH handle is created per Install so a worker whose Join timed
    // out (2s) in a previous Uninstall keeps observing its OWN Stop flag (still true forever) and can never
    // resurrect when a later Install would otherwise flip a shared flag back to false.
    private sealed class WorkerHandle
    {
        public          Thread?              Thread;
        public volatile bool                 Stop;
        public readonly ManualResetEventSlim Wake = new(false);
    }

    private WorkerHandle? _worker;
    private long          _workerPasses;     // Volatile — diagnostics
    private long          _workerLastMicros; // Volatile — diagnostics

    // Map bake.
    private OpacityFilter _opacityFilter = new();
    private string        _bakeDir       = string.Empty;
    private int           _mapGeneration; // Interlocked/Volatile — invalidates a slow bake after a map change

    // Weapon owner tracking: weapon entity index → owner controller index. Native auto-unhooks on deletion.
    private readonly Dictionary<EntityIndex, EntityIndex> _weaponOwner = new();
    private readonly HashSet<EntityIndex>                 _weaponSeen  = new();

    // Per-weapon consecutive-unseen frame counter (weapon index → misses). Debounces the gone-sweep un-link so a
    // one-frame snapshot hiccup can't blink a still-owned weapon's owner to Invalid (floating-gun flash). Only
    // holds weapons currently mid-miss — empty in steady state.
    private readonly Dictionary<EntityIndex, int>         _weaponUnseenFrames = new();

    // Ownership tracking so Uninstall only ever removes hooks WE added (never a co-resident transmit plugin's).
    private readonly bool[]               _weHooked        = new bool[Slots]; // slot-indexed: we hooked this controller
    private readonly HashSet<EntityIndex> _weHookedWeapons = new();           // weapons we (not someone else) hooked

    // Manual 2-client transmit test (temporary — see fow_test handler). While active the oracle is suspended
    // so a manually-forced SetEntityState sticks for observation. Admin-only + auto-expires (30s) so it can
    // never grief-freeze a hidden enemy permanently.
    private bool        _manualTestMode;
    private Guid        _manualTestTimer;
    private EntityIndex _manualForcedSender   = EntityIndex.InvalidIndex;
    private EntityIndex _manualForcedReceiver = EntityIndex.InvalidIndex;

    private Action<bool, bool, bool>? _frameHook;
    private int  _lastSlowTick;
    private int  _lastStatsTick;
    private int  _lastThrowLogTick;
    private int  _statFrames;
    private bool _installed;

    public FogOfWarModule(InterfaceBridge bridge, FogOfWarConfig config, ILogger logger)
    {
        _bridge = bridge;
        _config = config;
        _logger = logger;
    }

    public bool IsInstalled => _installed;

    public void Install()
    {
        // Re-entry guard: a second Install without an intervening Uninstall would leak a second worker thread /
        // double-register the listeners. Idempotent no-op if already installed.
        if (_installed)
            return;

        var cfg = _config;
        if (!cfg.Enabled)
        {
            _logger.LogInformation("[FogOfWar] disabled (opt-in) — not installing");
            return;
        }

        _channel      = Math.Clamp(cfg.TransmitChannel, 0, 5);
        _peekMargin   = MathF.Max(0f, cfg.PeekMarginUnits);
        _maxPredSpeed = MathF.Max(1f, cfg.MaxPredictionSpeed);
        _holdTicks    = MsToTicks(cfg.VisibilityHoldMs);
        _workerIntervalMs = Math.Max(5, cfg.UpdateIntervalMs);
        _slowTicks    = MsToTicks(SlowMs);

        // staleVisibleMs must comfortably EXCEED the worker cadence: it is the fail-open freshness bound, so if
        // it is <= updateIntervalMs (+ scheduling margin) even a normally-paced worker's newest snapshot is
        // always judged "stale" → FOW is permanently all-visible. Clamp up (2 intervals + 50ms margin) and warn.
        var minStaleMs = (_workerIntervalMs * 2) + 50;
        var staleMs    = cfg.StaleVisibleMs;
        if (staleMs < minStaleMs)
        {
            _logger.LogWarning(
                "[FogOfWar]: staleVisibleMs ({Stale}) too small vs updateIntervalMs ({Interval}) — "
                + "raising to {Min}ms so a normally-paced worker isn't treated as stalled (would force FOW all-visible)",
                cfg.StaleVisibleMs, cfg.UpdateIntervalMs, minStaleMs);
            staleMs = minStaleMs;
        }
        _staleTicks = MsToTicks(staleMs);
        _hideWeapons  = cfg.HideWeapons;
        _debugStatus  = cfg.DebugStatus;

        // lookahead_s = clamp(max(assumedRttMs + updateIntervalMs, minLookaheadMs), 0, maxLookaheadMs) / 1000.
        var raw = cfg.AssumedRttMs + cfg.UpdateIntervalMs;
        var clampedMs = Math.Clamp(Math.Max(raw, cfg.MinLookaheadMs), 0, Math.Max(cfg.MinLookaheadMs, cfg.MaxLookaheadMs));
        _lookaheadSeconds = clampedMs / 1000f;

        // Union the built-in see-through set with any operator-supplied surfaceprops (a custom map's translucent
        // material that would otherwise bake as an occluder → a false-hide). The bake logs the top included
        // surface-props so a live false-hide is diagnosable and the offending name can be added to config.
        _opacityFilter = new OpacityFilter(cfg.ExtraSeeThroughProps);
        _bakeDir       = Path.Combine(_bridge.SharpPath, "data", "fogofwar", "fow");

        _bridge.ClientManager.InstallClientListener(this);

        // Entity listener: evict a deleted weapon's tracking immediately so a recycled index can never make the
        // weapon gone-path poke a foreign hook (SetEntityOwner on a reused index).
        _bridge.EntityManager.InstallEntityListener(this);

        // Game listener: OnServerActivate fires on every map start → (re)bake that map's BVH off-thread.
        _bridge.ModSharp.InstallGameListener(this);

        _frameHook = OnFrame;
        _bridge.ModSharp.InstallGameFrameHook(null, _frameHook);

        // Start the background visibility worker BEFORE kicking the bake — it idles (fail-open) until _los is set.
        // A fresh handle per Install (see WorkerHandle): the worker checks its own Stop flag, so an orphaned
        // previous worker can never be revived by this start.
        var worker = new WorkerHandle();
        worker.Thread = new Thread(() => WorkerLoop(worker))
        {
            Name         = "FogOfWar",
            IsBackground = true,
        };
        _worker = worker;
        worker.Thread.Start();

        _installed = true;

        // Bake the CURRENT map now + hot-reload hook already-connected players. This is a MID-MAP-install
        // optimization only: on a COLD server boot GetGlobals() isn't ready yet (ModSharp throws "You can not
        // get this now!"), so guard it — OnServerActivate bakes the first map and OnClientPutInServer hooks
        // players as they connect, so a not-ready-globals boot must never abort Install (the worker + listeners
        // above are already live).
        try
        {
            KickBake(_bridge.ModSharp.GetGlobals().MapName);

            var max = Math.Min(Slots, _bridge.ModSharp.GetGlobals().MaxClients);
            for (var slot = 0; slot < max; slot++)
            {
                if (_bridge.EntityManager.FindPlayerControllerBySlot(new PlayerSlot((byte) slot)) is
                    { ConnectedState: PlayerConnectedState.PlayerConnected } controller)
                {
                    EnsureHooked(controller);
                }
            }
        }
        catch (Exception e)
        {
            _logger.LogInformation(
                "[FogOfWar]: globals not ready at Install — deferring first bake/hook to map activate ({Reason})",
                e.Message);
        }

        _logger.LogInformation(
            "[FogOfWar] ACTIVE (DENIAL, not gated by detectionMode) — BVH worker oracle — "
            + "channel={Channel}, lookahead={Lookahead}ms, peekMargin={Peek}, maxPredSpeed={MaxSpeed}, hold={Hold}t, "
            + "stale={Stale}t, workerInterval={Worker}ms, slowPass={Slow}t, hideWeapons={Weapons}, debug={Debug}, "
            + "bakeDir={BakeDir}",
            _channel, clampedMs, _peekMargin, _maxPredSpeed, _holdTicks, _staleTicks, _workerIntervalMs, _slowTicks,
            _hideWeapons, _debugStatus, _bakeDir);
    }

    public void Uninstall()
    {
        if (!_installed)
            return;

        _installed = false;

        // Stop + join the worker thread first (it only touches the immutable snapshots + its own wake, never any
        // ModSharp state), so nothing races the teardown below. The handle is per-run: even if Join times out,
        // the orphaned worker keeps seeing its own Stop=true and exits itself — a later Install cannot revive it.
        var worker = _worker;
        _worker = null;
        if (worker is not null)
        {
            worker.Stop = true;
            worker.Wake.Set();
            try { worker.Thread?.Join(2000); } catch { /* best-effort */ }
        }

        // Invalidate any in-flight bake task so it won't publish a stale BVH after teardown.
        Interlocked.Increment(ref _mapGeneration);
        Volatile.Write(ref _losState, null);
        Volatile.Write(ref _vis, null);
        Volatile.Write(ref _input, null);

        if (_frameHook is not null)
        {
            _bridge.ModSharp.RemoveGameFrameHook(null, _frameHook);
            _frameHook = null;
        }

        _bridge.ClientManager.RemoveClientListener(this);
        _bridge.EntityManager.RemoveEntityListener(this);
        _bridge.ModSharp.RemoveGameListener(this);

        // Cancel any pending manual-test auto-expire (force-calls EndManualTest → clears mode, re-asserts visible).
        if (_manualTestMode || _manualTestTimer != Guid.Empty)
            CancelManualTest();

        // Never leave hidden state behind — but only unhook controllers / weapons WE added, never a co-resident
        // transmit plugin's hook (IsEntityHooked is global, so removing an unowned hook would break VIP-invis etc.).
        // Iterate all slots (const) rather than GetGlobals().MaxClients — on Shutdown the globals can be
        // unavailable (ModSharp: "You can not get this now!"), and an empty slot just resolves to a null
        // controller below and is skipped. Avoids throwing during teardown.
        for (var slot = 0; slot < Slots; slot++)
        {
            if (!_weHooked[slot])
                continue;
            if (_bridge.EntityManager.FindPlayerControllerBySlot(new PlayerSlot((byte) slot)) is { } controller
                && _bridge.TransmitManager.IsEntityHooked(controller))
                _bridge.TransmitManager.RemoveEntityHooks(controller);
        }

        foreach (var widx in _weHookedWeapons)
        {
            if (_bridge.EntityManager.FindEntityByIndex(widx) is { } weapon
                && _bridge.TransmitManager.IsEntityHooked(weapon))
                _bridge.TransmitManager.RemoveEntityHooks(weapon);
        }

        Array.Clear(_weHooked);
        _weHookedWeapons.Clear();

        Array.Clear(_revealedUntil);
        Array.Clear(_pushedHidden);
        Array.Clear(_snap);

        // Reset the native owner linkage of every tracked weapon before dropping belief. Clearing only our
        // dictionary would leave weapons owner-linked to a now-dead FOW linkage on a co-resident transmit plugin's
        // hook (weapons WE owner-set but did NOT hook are never RemoveEntityHooks'd above), so that plugin inherits
        // the desync after we unload. A stale weapon index fails gracefully. Purely un-links → fail-open.
        foreach (var widx in _weaponOwner.Keys)
            _bridge.TransmitManager.SetEntityOwner(widx, EntityIndex.InvalidIndex);
        _weaponOwner.Clear();
        _weaponUnseenFrames.Clear();
        _weaponSeen.Clear();
        _manualTestMode       = false;
        _manualForcedSender   = EntityIndex.InvalidIndex;
        _manualForcedReceiver = EntityIndex.InvalidIndex;
    }

    // ── Game frame (post, game thread) ───────────────────────────────────────────────────────────────────
    private void OnFrame(bool simulating, bool first, bool last)
    {
        // Only cull while the server is actually simulating a tick.
        if (!_installed || !simulating)
            return;

        // Manual transmit test suspends the whole automatic oracle (including the fail-open enforcement below)
        // so a forced SetEntityState stays observable. This is admin-only and auto-expires (30s), so it can
        // never freeze a hidden enemy indefinitely — see HandleFowTest / EndManualTest.
        if (_manualTestMode)
            return;

        var now = 0;
        var max = 0;
        var oracleThrew = false;
        try
        {
            now = _bridge.ModSharp.GetGlobals().TickCount;
            max = Math.Min(Slots, _bridge.ModSharp.GetGlobals().MaxClients);

            Snapshot(max, now);
            PublishWorkerInput(max, now); // hand game → worker (immutable, volatile publish)

            var slowTick = now - _lastSlowTick >= _slowTicks;
            if (slowTick)
            {
                _lastSlowTick = now;
                RehookSweep(max, now);
            }

            var vis = Volatile.Read(ref _vis); // worker → game (immutable, volatile read)
            ApplyTransitions(max, now, vis, forceAll: slowTick);

            // Weapon reconcile runs EVERY frame (NOT the 250ms slow tick): the pawn hides/reveals per-frame, so
            // a weapon owner-linked only every 250ms floats visible (owner unset) for up to ~250ms after a
            // pickup/switch while its pawn is hidden — the "floating gun, no player" desync. Per-frame locks the
            // weapon's hidden state to its owner (window ≤ 1 frame). The scan is kept light by a dirty-check
            // inside ReconcileWeapons: the native transmit calls (hook + SetEntityOwner) fire ONLY for a weapon
            // new to us or whose owner changed; a steady-state weapon costs one dictionary lookup. So the residual
            // per-frame work is just the O(players×weapons) inventory walk, well under the O(players²) pair apply.
            if (_hideWeapons)
                ReconcileWeapons(max);

            if (_debugStatus)
            {
                _statFrames++;
                if (now - _lastStatsTick >= 60 * TickRate)
                {
                    _lastStatsTick = now;
                    LogStats(max);
                }
            }
        }
        catch (Exception e)
        {
            oracleThrew = true;
            // Throttle so a persistently-throwing frame can't spam the log. A swallowed throw here would
            // otherwise freeze hides — the operator must see it.
            if (now - _lastThrowLogTick >= ThrowLogCooldownTicks)
            {
                _lastThrowLogTick = now;
                _logger.LogWarning(e, "[FogOfWar] frame threw — forcing all hidden pairs visible (fail-open)");
            }
        }
        finally
        {
            // FAIL-OPEN ENFORCEMENT — MUST run even when the game-thread apply above throws, or a hidden enemy
            // could stay frozen invisible. On a clean frame ApplyTransitions already asserted the correct state;
            // on a throw we release every pair we currently believe we've hidden (push VISIBLE).
            if (oracleThrew)
                ForceAllVisible();
        }
    }

    // Fail-open fallback: release every pair we currently believe we've hidden by pushing VISIBLE. Used when the
    // game-thread apply throws, so a bug can never leave a hidden enemy frozen invisible. Controller indices are
    // derived from the slot (not the possibly-stale snapshot) since the snapshot may be what threw.
    private void ForceAllVisible()
    {
        for (var idx = 0; idx < _pushedHidden.Length; idx++)
        {
            if (!_pushedHidden[idx])
                continue;

            var receiverSlot = idx / Slots;
            var senderSlot   = idx % Slots;
            var senderCtrl   = new EntityIndex(new PlayerSlot((byte) senderSlot));
            var receiverCtrl = new EntityIndex(new PlayerSlot((byte) receiverSlot));
            if (_bridge.TransmitManager.SetEntityState(senderCtrl, receiverCtrl, transmit: true, _channel))
                _pushedHidden[idx] = false;
        }
    }

    // ── Snapshot: capture live pawn state once per frame ─────────────────────────────────────────────────
    private void Snapshot(int max, int now)
    {
        for (var slot = 0; slot < max; slot++)
        {
            ref var s = ref _snap[slot];

            if (_bridge.EntityManager.FindPlayerControllerBySlot(new PlayerSlot((byte) slot)) is not
                { ConnectedState: PlayerConnectedState.PlayerConnected } controller)
            {
                s = default;
                continue;
            }

            var ctrlIndex = controller.Index;

            if (controller.GetPlayerPawn() is not { IsAlive: true } pawn)
            {
                // Connected but dead / spectating → not a valid receiver or sender (dead pawns are never
                // culled, and a spectator's chase-cam must see through the eyes of whoever they watch).
                s = new PawnSnap { Valid = false, CtrlIndex = ctrlIndex };
                continue;
            }

            Vector mins, maxs;
            if (pawn.GetCollisionProperty() is { } cp)
            {
                mins = cp.Mins;
                maxs = cp.Maxs;
            }
            else
            {
                // Standard CS2 standing player hull fallback.
                mins = new Vector(-16f, -16f, 0f);
                maxs = new Vector(16f, 16f, 72f);
            }

            s = new PawnSnap
            {
                Valid     = true,
                Human     = !controller.IsFakeClient, // bots excluded as receivers; HLTV has no alive pawn
                Ducked    = (pawn.Flags & EntityFlags.Ducking) != 0, // A7: un-crouch peek origin o4
                Team      = controller.Team,
                Eye       = pawn.GetEyePosition(),
                Origin    = pawn.GetAbsOrigin(),
                Velocity  = pawn.GetAbsVelocity(),
                Mins      = mins,
                Maxs      = maxs,
                CtrlIndex = ctrlIndex,
            };
        }
    }

    // ── Publish the current player snapshot to the worker (game → worker hand-off) ───────────────────────
    // Converts the live ModSharp snapshot into a fresh immutable, ModSharp-free struct array (System.Numerics
    // only) and volatile-publishes it. The worker reads whatever the latest published input is.
    private void PublishWorkerInput(int max, int now)
    {
        var players = new LosSampler.SampledPlayer[Slots];
        for (var slot = 0; slot < max; slot++)
        {
            ref var s = ref _snap[slot];
            if (!s.Valid)
            {
                players[slot] = default;
                continue;
            }

            players[slot] = new LosSampler.SampledPlayer
            {
                Valid    = true,
                Human    = s.Human,
                Ducked   = s.Ducked,
                Team     = (int) s.Team,
                Eye      = ToV3(s.Eye),
                Origin   = ToV3(s.Origin),
                Velocity = ToV3(s.Velocity),
                Mins     = ToV3(s.Mins),
                Maxs     = ToV3(s.Maxs),
            };
        }

        // Stamp the current map generation so the worker never pairs these positions with a different map's BVH.
        Volatile.Write(ref _input, new WorkerInput(players, max, now, Volatile.Read(ref _mapGeneration)));
    }

    private static Vector3 ToV3(Vector v) => new(v.X, v.Y, v.Z);

    // ── Background worker: recompute the full visibility matrix off the game thread ──────────────────────
    private void WorkerLoop(WorkerHandle h)
    {
        var sw = new Stopwatch();
        // Off-thread throttle for the compute-fault warning (mirrors the game-thread ThrowLogCooldownTicks 5s
        // window, but wall-clock since the worker must never read the game's TickCount).
        var lastThrowLogMs = 0L;

        // Worker-private temporal-coherence hint buffer: per enemy pair, the ray index that last cleared LOS, so
        // the next pass tests it FIRST. An OR-of-clears sweep is order-independent, so this is a pure early-out —
        // it can never change the boolean matrix. Reset to the "no hint" sentinel whenever the map generation
        // changes so a hint never carries into a different map's geometry.
        var lastClear    = new byte[Slots * Slots];
        var lastCacheGen = -1;

        while (!h.Stop)
        {
            sw.Restart();

            var input = Volatile.Read(ref _input);
            var los   = Volatile.Read(ref _losState); // atomic (BVH + generation) pair — never torn across a map change
            var gen   = Volatile.Read(ref _mapGeneration);

            // Only compute when the positions, the geometry AND the current map generation all agree — refuse to
            // pair old-map positions (stale _input) with the new map's BVH, which would produce a garbage matrix
            // the game thread might briefly accept at map start (negative/near-zero staleness) → false-hide window.
            if (input is not null && los is not null && input.Generation == gen && los.Generation == gen)
            {
                // A map change (BVH swap) invalidates every remembered ray index — a hint index that cleared
                // against the OLD map's geometry has no relation to the new one. Reset (not just skip) so a stale
                // hint never silently short-circuits against the wrong geometry.
                if (lastCacheGen != gen)
                {
                    Array.Fill(lastClear, LosSampler.NoClearHint);
                    lastCacheGen = gen;
                }

                try
                {
                    var blocked = new bool[Slots * Slots];
                    LosSampler.ComputeMatrix(
                        input.Players, input.Max, los.Query,
                        _lookaheadSeconds, _peekMargin, _maxPredSpeed, blocked, lastClear);
                    Volatile.Write(ref _vis, new VisSnapshot(blocked, input.Tick, los.Generation));

                    Volatile.Write(ref _workerLastMicros, sw.ElapsedTicks * 1_000_000L / Stopwatch.Frequency);
                    Volatile.Write(ref _workerPasses, Volatile.Read(ref _workerPasses) + 1);
                }
                catch (Exception e)
                {
                    // A compute fault must not stall the loop — publish nothing new; the game thread fail-opens
                    // on the now-ageing snapshot (stale → all visible). But a worker that throws EVERY pass would
                    // leave FOW permanently all-visible with zero signal, so log it (throttled to 5s).
                    var nowMs = Environment.TickCount64;
                    if (nowMs - lastThrowLogMs >= ThrowLogCooldownMs)
                    {
                        lastThrowLogMs = nowMs;
                        _logger.LogWarning(e,
                            "[FogOfWar] worker compute threw — publishing no new snapshot; game thread "
                            + "fail-opens on the stale snapshot (all visible)");
                    }
                }
            }

            if (h.Stop)
                break;

            // Pace to the configured cadence (floor 1ms to never spin the CPU). The wake is only set on shutdown.
            var elapsedMs = (int) sw.ElapsedMilliseconds;
            var waitMs    = Math.Max(1, _workerIntervalMs - elapsedMs);
            if (h.Wake.Wait(waitMs))
                h.Wake.Reset();
        }
    }

    // ── Map bake (off the game thread) ────────────────────────────────────────────────────────────────────
    // Reset the ready state (→ fail-open) and kick an async bake for the given map. Called on the game thread
    // (Install for the current map, OnServerActivate for a map change).
    private void KickBake(string mapName)
    {
        var gen = Interlocked.Increment(ref _mapGeneration);
        Volatile.Write(ref _losState, null); // FOW fully fail-open (all visible) until the new bake is ready
        Volatile.Write(ref _vis, null);
        // Drop the old map's player positions too: a cache-hit new-map BVH paired with stale _input could make
        // the worker publish a garbage matrix under the new generation before the first new-map frame republishes.
        Volatile.Write(ref _input, null);

        // Map change: the old map's entities/hooks are gone (engine default = VISIBLE) and TickCount may reset to
        // 0. Zero the tick trackers so the slow / stats / throw-log cadences don't go dormant on the new map (a
        // stale large _lastSlowTick would make `now - _lastSlowTick` negative → the 250ms slow pass — RehookSweep
        // and the full re-assert in ApplyTransitions — never runs), and drop all per-pair belief (nothing to
        // un-hide — the new map's pawns default visible). Game-thread only; safe here.
        _lastSlowTick     = 0;
        _lastStatsTick    = 0;
        _lastThrowLogTick = 0;
        Array.Clear(_revealedUntil);
        Array.Clear(_pushedHidden);

        // Drop stale "we hooked this controller" ownership beliefs: on a map change the old controllers are
        // destroyed (native auto-unhooks) and the same slots are reused for fresh controllers a co-resident
        // transmit plugin may hook first. Without this, a later Uninstall's RemoveEntityHooks could rip a hook
        // we never actually own on the new map. RehookSweep/EnsureHooked re-establishes real ownership.
        Array.Clear(_weHooked);

        var gamePath = _bridge.ModSharp.GetGamePath(); // game thread — resolve here, don't touch ModSharp off-thread
        Task.Run(() => BakeMap(mapName, gamePath, gen));
    }

    private void BakeMap(string mapName, string gamePath, int gen)
    {
        try
        {
            var vpk = ResolveMapVpk(gamePath, mapName);
            if (vpk is null)
            {
                _logger.LogWarning(
                    "[FogOfWar]: no map .vpk found for '{Map}' under '{GamePath}' — FOW stays fail-open (all visible)",
                    mapName, gamePath);
                return;
            }

            // Cheap key read — reuses the map-name-aware candidate selection WITHOUT extracting the nested vpk.
            if (!MapCollisionLoader.TryGetPhysicsKey(vpk, mapName, out var crc, out var size))
            {
                _logger.LogWarning(
                    "[FogOfWar]: could not read world_physics key from '{Vpk}' — FOW stays fail-open", vpk);
                return;
            }

            LosQuery los;
            if (BakeCache.TryLoad(_bakeDir, crc, size, out var cached) && cached is not null)
            {
                los = cached.WithSimd();
                _logger.LogInformation(
                    "[FogOfWar]: loaded baked BVH for '{Map}' from cache — {Tris} tris, {Nodes} nodes, crc={Crc:x8}",
                    mapName, los.TriangleCount, los.NodeCount, crc);
            }
            else
            {
                var sw = Stopwatch.StartNew();
                // Fully extract only on the miss path; map-name-aware selection picks the right world_physics.
                var geom = MapCollisionLoader.Load(vpk, _opacityFilter, mapName);

                // A vpk packing more than one world_physics (multi-map addon / stub + real) is exactly where a
                // wrong-geometry false-hide could hide — surface the ambiguity + which one we chose.
                if (geom.Stats.KeyCandidateCount > 1 || geom.Stats.WorldPhysicsCandidateCount > 1)
                    _logger.LogWarning(
                        "[FogOfWar]: '{Map}' had multiple world_physics candidates "
                        + "(keyCandidates={Keys}, inPackage={Inner}); chose '{Path}' — verify this is the right map's geometry",
                        mapName, geom.Stats.KeyCandidateCount, geom.Stats.WorldPhysicsCandidateCount,
                        geom.Stats.ChosenPath);

                // Stub guard: an implausibly small occluder set is a bake failure (placeholder / wrong pick), not
                // a real map. Fail open (all visible) rather than baking a stub that would silently under-hide.
                if (geom.TriangleCount < MinReasonableTriangles)
                {
                    _logger.LogWarning(
                        "[FogOfWar]: '{Map}' world_physics '{Path}' has only {Tris} occluder triangles "
                        + "(< {Min}) — treating as a bake failure; FOW stays fail-open (all visible)",
                        mapName, geom.Stats.ChosenPath, geom.TriangleCount, MinReasonableTriangles);
                    return;
                }

                los = LosQuery.BuildWithSimd(geom);
                BakeCache.Save(_bakeDir, los);
                _logger.LogInformation(
                    "[FogOfWar]: baked BVH for '{Map}' from '{Path}' — {Incl} occluder tris "
                    + "({ExclLayer} excluded by interaction-layer + {Excl} by surfaceprop see-through), "
                    + "{Nodes} nodes, crc={Crc:x8}, {Ms}ms; cached to {Dir}",
                    mapName, geom.Stats.ChosenPath, geom.Stats.IncludedTriangles, geom.Stats.ExcludedByOpacityLayer,
                    geom.Stats.ExcludedSeeThrough, los.NodeCount, crc, sw.ElapsedMilliseconds, _bakeDir);

                LogTopSurfaceProps(mapName, geom.Stats);
            }

            _logger.LogInformation(
                "[FogOfWar]: '{Map}' LOS engine = {Engine}",
                mapName, los.SimdEnabled ? los.Simd!.ActiveTier.ToString() : "scalar");

            // Publish bound to this bake's generation, but ONLY via a generation-ordered CAS so a slow OLD-map
            // bake that finishes AFTER a newer map's bake can never stomp the newer BVH (which would silently
            // pin the whole new map to fail-open all-visible). Retry on a racing publish; bail if superseded /
            // torn down or if a same-or-newer generation's BVH is already installed.
            var next = new LosState(los, gen);
            while (true)
            {
                if (Volatile.Read(ref _mapGeneration) != gen || !_installed)
                    break; // a newer map change (or teardown) has superseded this bake

                var current = Volatile.Read(ref _losState);
                if (current is not null && current.Generation >= gen)
                    break; // a same-or-newer bake already published → never regress it

                if (Interlocked.CompareExchange(ref _losState, next, current) == current)
                    break; // installed — worker picks it up within one interval
                // else: _losState changed under us (racing publish / map-change null) → re-evaluate and retry
            }
        }
        catch (Exception e)
        {
            _logger.LogWarning(e,
                "[FogOfWar]: bake failed for '{Map}' — FOW stays fail-open (all visible)", mapName);
        }
    }

    // Resolve the on-disk .vpk for a map name. Official maps and most workshop maps live in csgo/maps/<name>.vpk
    // (workshop compiles are packed as <name>_dir.vpk, sometimes under maps/workshop/<id>/). Returns null if not
    // found → FOW stays fail-open for that map.
    private static string? ResolveMapVpk(string gamePath, string mapName)
    {
        if (string.IsNullOrWhiteSpace(gamePath) || string.IsNullOrWhiteSpace(mapName))
            return null;

        // MapName is usually a bare name, but strip any path prefix defensively.
        var name  = mapName;
        var slash = name.LastIndexOfAny(new[] { '/', '\\' });
        if (slash >= 0)
            name = name[(slash + 1)..];
        if (name.Length == 0)
            return null;

        // GetGamePath() may return the game root (contains csgo/) or the csgo/ content dir directly.
        var roots = new[]
        {
            Path.Combine(gamePath, "csgo", "maps"),
            Path.Combine(gamePath, "maps"),
        };

        var fileNames = new[] { name + ".vpk", name + "_dir.vpk" };

        foreach (var root in roots)
        {
            if (!Directory.Exists(root))
                continue;

            foreach (var fn in fileNames)
            {
                var direct = Path.Combine(root, fn);
                if (File.Exists(direct))
                    return direct;
            }

            var workshop = Path.Combine(root, "workshop");
            if (!Directory.Exists(workshop))
                continue;

            foreach (var fn in fileNames)
            {
                foreach (var hit in Directory.EnumerateFiles(workshop, fn, SearchOption.AllDirectories))
                    return hit;
            }
        }

        return null;
    }

    // Log the top-10 INCLUDED (baked-as-occluder) surface-property names + triangle counts. If a live false-hide
    // is reported (an enemy behind a translucent surface wrongly culled), this pins the candidate surfaceprop to
    // add to fogOfWar.extraSeeThroughProps — turning an otherwise-undiagnosable false-hide into a config fix.
    private void LogTopSurfaceProps(string mapName, LoadStats stats)
    {
        if (stats.IncludedByProp.Count == 0)
            return;

        // Top 10 always; with debugStatus, dump EVERY included surfaceprop so a fence/grate baked as an occluder
        // (the false-hide culprit) can be identified by name for fogOfWar.extraSeeThroughProps.
        var take = _debugStatus ? int.MaxValue : 10;
        var top = string.Join(", ", stats.IncludedByProp
            .OrderByDescending(kv => kv.Value)
            .Take(take)
            .Select(kv => $"{kv.Key}={kv.Value}"));

        _logger.LogInformation(
            "[FogOfWar]: '{Map}' included surface-props (add a false-hiding one to "
            + "fogOfWar.extraSeeThroughProps): {Props}",
            mapName, top);
    }

    // ── Re-hook sweep (slow cadence): self-heal native hook loss (map change, slot reuse) ────────────────
    private void RehookSweep(int max, int now)
    {
        for (var slot = 0; slot < max; slot++)
        {
            if (_bridge.EntityManager.FindPlayerControllerBySlot(new PlayerSlot((byte) slot)) is not
                { ConnectedState: PlayerConnectedState.PlayerConnected } controller)
                continue;

            if (!_bridge.TransmitManager.IsEntityHooked(controller))
            {
                if (_bridge.TransmitManager.AddEntityHooks(controller, true))
                {
                    _weHooked[slot] = true;
                    ResetSlotPairs(slot, now); // fresh hook defaults visible → drop any stale "we pushed hidden" belief
                }
            }
        }
    }

    // ── Apply transitions: derive visibility (with fail-open) and push state changes to the engine ────────
    private void ApplyTransitions(int max, int now, VisSnapshot? vis, bool forceAll)
    {
        // Fail-open staleness: no worker snapshot yet (worker not started / bake not loaded), the newest one is
        // for a superseded map (generation mismatch — a pass that finished against the previous map's BVH across
        // a map change), it is older than staleVisibleMs (worker stalled), OR its InputTick is in the FUTURE
        // (now < InputTick — a snapshot computed from a previous map's positions after TickCount reset at map
        // start; a plain `now - InputTick` would be negative and wrongly pass the staleness test) → force EVERY
        // pair visible this frame.
        var visStale = vis is null
                       || vis.Generation != Volatile.Read(ref _mapGeneration)
                       || now < vis.InputTick
                       || now - vis.InputTick > _staleTicks;

        for (var r = 0; r < max; r++)
        {
            ref var rs = ref _snap[r];
            var receiverOk = rs.Valid && rs.Human;

            for (var sSlot = 0; sSlot < max; sSlot++)
            {
                if (sSlot == r)
                    continue;

                var idx = (r * Slots) + sSlot;
                ref var ss = ref _snap[sSlot];

                // Per-push re-validation (§1.2): only a live human receiver vs a live STRICTLY-OPPOSING (T-vs-CT)
                // sender is ever a hide candidate. Everything else (dead / spectating / same team / None /
                // Spectator team / bot receiver / missing snapshot) FORCES VISIBLE.
                var candidate = receiverOk && ss.Valid && AreEnemies(rs.Team, ss.Team);

                bool desiredHide;
                if (!candidate || visStale)
                {
                    // Non-candidate, or the whole matrix is stale/missing → fail-open VISIBLE.
                    desiredHide = false;

                    // Hysteresis on the fail-open reveal too. The oracle-clear path below latches _revealedUntil
                    // each frame a pair is seen visible; these NON-oracle forced-visible paths (visStale worker
                    // recovery, candidate→non-candidate transition) previously released a hidden pair WITHOUT the
                    // hold, so the instant the worker recovered one interval later with a still-blocked matrix the
                    // pair re-hid on the very next frame — a server-wide strobe under rhythmic load. Latch the hold
                    // as we release so recovery can't re-hide inside the window. Monotone (only ADDS visible time).
                    if (_pushedHidden[idx])
                        _revealedUntil[idx] = now + _holdTicks;
                }
                else
                {
                    // Symmetric visibility (server-owner request): between two real players, reveal the pair if
                    // EITHER direction has LOS — hide only when BOTH r→s AND s→r are blocked, so "if one sees
                    // the other, the other sees him too". Blocked[a*Slots + b] = "observer a (eye) cannot see
                    // target b (box)" (see ComputeMatrix / PairBlocked: pairIdx = r*Slots + s), so idx = r→sSlot
                    // and the mirror cell is sSlot→r = [(sSlot * Slots) + r].
                    //   BUT the mirror is only valid when the SENDER is a real human: ComputeMatrix never samples
                    // a bot/non-human RECEIVER row (it stays false = "visible"), so an unconditional AND would
                    // wrongly reveal a bot target through walls (breaks the anti-wallhack + bot testing). So gate
                    // the mirror on ss.Human: a bot target is hidden one-directionally (on the human receiver's
                    // LOS alone); a human target uses the symmetric rule.
                    //   Monotone either way: (blocked[idx] AND mirror) ⊆ blocked[idx] → only ever ADDS visibility.
                    var mirrorBlocked = !ss.Human || vis!.Blocked[(sSlot * Slots) + r];
                    var blocked       = vis!.Blocked[idx] && mirrorBlocked;
                    if (!blocked)
                        _revealedUntil[idx] = now + _holdTicks; // hysteresis: latch the hold each frame seen visible

                    var visible = !blocked || now < _revealedUntil[idx];
                    desiredHide = !visible;
                }

                // On a NORMAL tick: skip pairs needing no change. On the forceAll (slow ~250ms) tick: re-assert
                // the WHOLE live matrix — every candidate's desired state (VISIBLE as well as HIDDEN), plus a
                // release of any non-candidate pair we still believe we've hidden — so that if another transmit
                // plugin stomps our channel, a wrongly-hidden enemy is re-pushed VISIBLE and self-heals. The only
                // pairs skippable on forceAll are non-candidates we've never touched (engine default is visible).
                if (forceAll)
                {
                    if (!candidate && !_pushedHidden[idx])
                        continue;
                }
                else
                {
                    if (!desiredHide && !_pushedHidden[idx])
                        continue;
                    if (desiredHide == _pushedHidden[idx])
                        continue;
                }

                // transmit = visible (true). Hide by sending false. sender pawn is cleared for this receiver.
                var ok = _bridge.TransmitManager.SetEntityState(
                    ss.CtrlIndex, rs.CtrlIndex, transmit: !desiredHide, _channel);

                if (ok)
                {
                    _pushedHidden[idx] = desiredHide;
                }
                else if (candidate && desiredHide)
                {
                    // Hook missing on the sender controller → can't hide; leave belief unchanged (retry) and
                    // trigger a re-hook. Fail-open: the enemy simply stays visible in the meantime.
                    if (_bridge.EntityManager.FindPlayerControllerBySlot(new PlayerSlot((byte) sSlot)) is
                        { ConnectedState: PlayerConnectedState.PlayerConnected } controller
                        && !_bridge.TransmitManager.IsEntityHooked(controller))
                    {
                        if (_bridge.TransmitManager.AddEntityHooks(controller, true))
                        {
                            _weHooked[sSlot] = true;
                            ResetSlotPairs(sSlot, now);
                        }
                    }
                }
            }
        }
    }

    // ── Weapon reconcile (per frame, dirty-checked): hide carried weapons with their owner ───────────────
    private void ReconcileWeapons(int max)
    {
        _weaponSeen.Clear();

        for (var slot = 0; slot < max; slot++)
        {
            ref var s = ref _snap[slot];
            if (!s.Valid)
                continue;

            if (_bridge.EntityManager.FindPlayerControllerBySlot(new PlayerSlot((byte) slot)) is not
                { ConnectedState: PlayerConnectedState.PlayerConnected } controller)
                continue;

            if (controller.GetPlayerPawn() is not { IsAlive: true } pawn
                || pawn.GetWeaponService() is not { } ws)
                continue;

            var list = ws.GetMyWeapons();
            for (var i = 0; i < list.Count; i++)
            {
                var handle = list[i];
                if (!handle.IsValid())
                    continue;

                if (_bridge.EntityManager.FindEntityByHandle(handle) is not { } weapon)
                    continue;

                var widx = weapon.Index;
                _weaponSeen.Add(widx); // record BEFORE the dirty-check skip so the gone-weapon sweep still sees it

                // DIRTY-CHECK (keeps the per-frame scan cheap): a weapon already hooked and owner-linked to THIS
                // same controller needs no native call — SetEntityOwner persists in the hook (GetEntityOwner reads
                // it back), so re-asserting an unchanged owner every frame was pure P/Invoke waste (the reason the
                // full per-frame reconcile was expensive). Only a weapon NEW to us, or one whose owner CHANGED
                // (buy / pickup / drop→pickup / switch / respawn), falls through to touch the transmit manager.
                // Steady state is a single dictionary lookup. Fail-open safe: if we ever wrongly skip, the weapon
                // just stays linked to a still-valid owner (never hides a visible pawn's weapon).
                if (_weaponOwner.TryGetValue(widx, out var linked) && linked == s.CtrlIndex)
                    continue;

                if (!_bridge.TransmitManager.IsEntityHooked(weapon))
                {
                    // Only claim ownership of hooks WE add — never a co-resident transmit plugin's.
                    if (_bridge.TransmitManager.AddEntityHooks(weapon, true))
                        _weHookedWeapons.Add(widx);
                }

                // A hooked weapon whose owner controller is hidden from a receiver is cleared with it — so
                // this single owner assignment carries all the per-pair culling for free.
                _bridge.TransmitManager.SetEntityOwner(widx, s.CtrlIndex);
                _weaponOwner[widx] = s.CtrlIndex;
            }
        }

        // Weapons that left an inventory (drop / death) must become owner-less so they stop being culled — but
        // with a TWO-FRAME confirmation. A single-frame controller/pawn resolution hiccup (transient snapshot
        // invalidation) can momentarily drop a still-carried weapon out of _weaponSeen; un-linking on the FIRST
        // miss would flash a floating gun for one tick before re-linking next frame. Require 2 consecutive unseen
        // frames. Tradeoff: a genuinely-dropped gun un-links ~15ms later — that only keeps an owned-until-just-now
        // weapon linked to its still-hidden owner a frame longer, never hides a player → fail-open-permitted.
        List<EntityIndex>? gone = null;
        foreach (var kv in _weaponOwner)
        {
            if (_weaponSeen.Contains(kv.Key))
            {
                if (_weaponUnseenFrames.Count != 0)
                    _weaponUnseenFrames.Remove(kv.Key); // seen again → reset the miss streak
                continue;
            }

            var misses = _weaponUnseenFrames.GetValueOrDefault(kv.Key) + 1;
            if (misses >= 2)
                (gone ??= []).Add(kv.Key);
            else
                _weaponUnseenFrames[kv.Key] = misses;
        }

        if (gone is not null)
        {
            foreach (var widx in gone)
            {
                _bridge.TransmitManager.SetEntityOwner(widx, EntityIndex.InvalidIndex);
                _weaponOwner.Remove(widx);
                _weaponUnseenFrames.Remove(widx);
            }
        }
    }

    // ── Helpers ──────────────────────────────────────────────────────────────────────────────────────────

    private static int MsToTicks(int ms)
        => Math.Max(1, (int) MathF.Round(ms / 1000f * TickRate));

    /// <summary>
    ///     Strict T-vs-CT enemy test. A bare <c>rs.Team != ss.Team</c> would let an <c>UnAssigned</c> /
    ///     <c>Spectator</c> pawn become a hide candidate; only Terrorist-vs-CT (either direction) may ever hide.
    /// </summary>
    private static bool AreEnemies(CStrikeTeam a, CStrikeTeam b)
        => (a == CStrikeTeam.TE && b == CStrikeTeam.CT) || (a == CStrikeTeam.CT && b == CStrikeTeam.TE);

    private void EnsureHooked(IPlayerController controller)
    {
        // If it's already hooked, do NOT claim it as ours — a co-resident transmit plugin (VIP invis, etc.)
        // may own it, and Uninstall must never rip out a hook we didn't add. We can still push state on our own
        // channel through a foreign hook, so nothing is lost by not owning it.
        if (_bridge.TransmitManager.IsEntityHooked(controller))
            return;

        // NEVER trust AddEntityHooks blindly (old Prophunt bug) — log a failure to hook.
        if (_bridge.TransmitManager.AddEntityHooks(controller, true))
        {
            var slot = controller.PlayerSlot.AsPrimitive();
            if (slot < Slots)
                _weHooked[slot] = true;
        }
        else
        {
            _logger.LogWarning("[FogOfWar] failed to hook controller idx={Index}", controller.Index);
        }
    }

    /// <summary>
    ///     Reset all pair state where <paramref name="slot" /> is the sender OR the receiver. <paramref name="now" />
    ///     is the current tick for the hysteresis latch on a fresh-hook reveal (a re-added hook defaults VISIBLE, so
    ///     a pair we were hiding is released here); pass -1 (teardown / disconnect) to skip the latch and hard-clear.
    /// </summary>
    private void ResetSlotPairs(int slot, int now)
    {
        for (var other = 0; other < Slots; other++)
        {
            ClearPair((slot * Slots) + other, now); // slot as receiver
            ClearPair((other * Slots) + slot, now); // slot as sender
        }
    }

    private void ClearPair(int idx, int now)
    {
        // A fresh hook (RehookSweep / hook-retry) means the engine now defaults this pair VISIBLE. If we were
        // hiding it, latch the hysteresis hold as we drop belief so the recovering oracle can't instantly re-hide
        // it next frame (rehook strobe). Monotone — only adds visible time. now < 0 (teardown/disconnect) → clear.
        _revealedUntil[idx] = (now >= 0 && _pushedHidden[idx]) ? now + _holdTicks : 0;
        _pushedHidden[idx]  = false;
    }

    private void LogStats(int max)
    {
        var hidden     = 0;
        var candidates = 0;
        for (var r = 0; r < max; r++)
        {
            for (var s = 0; s < max; s++)
            {
                if (r == s)
                    continue;
                var idx = (r * Slots) + s;
                if (_pushedHidden[idx])
                    hidden++;
                if (r != s && _snap[r].Valid && _snap[s].Valid)
                    candidates++;
            }
        }

        var vis    = Volatile.Read(ref _vis);
        var los    = Volatile.Read(ref _losState);
        var passes = Volatile.Read(ref _workerPasses);
        var micros = Volatile.Read(ref _workerLastMicros);
        var ageT   = vis is null ? -1 : _bridge.ModSharp.GetGlobals().TickCount - vis.InputTick;

        _logger.LogInformation(
            "[FogOfWar] stats(60s) — bvhReady={Ready}({Tris} tris), workerPasses={Passes}, "
            + "~{Micros}us/pass, snapshotAge={Age}t, hiddenPairs={Hidden}, livePairs={Cand}, hookedWeapons={Weapons}",
            los is not null, los?.Query.TriangleCount ?? 0, passes, micros, ageT, hidden, candidates, _weaponSeen.Count);
    }

    // ── IGameListener (game thread) ───────────────────────────────────────────────────────────────────────
    // Map start: (re)bake the new map's BVH off-thread. Until it lands, FOW is fully fail-open (all visible).
    public void OnServerActivate()
    {
        if (!_installed)
            return;

        KickBake(_bridge.ModSharp.GetGlobals().MapName);
    }

    // ── IClientListener (game thread) ─────────────────────────────────────────────────────────────────────
    public void OnClientPutInServer(IGameClient client)
    {
        if (!_installed)
            return;

        // Hook the CONTROLLER after a short delay so all data is ready (docfx pattern). Re-resolve by slot
        // inside the timer — never store the client / controller across the delay.
        var slot = client.Slot.AsPrimitive();
        _bridge.ModSharp.PushTimer(() =>
        {
            if (!_installed)
                return;
            if (_bridge.EntityManager.FindPlayerControllerBySlot(new PlayerSlot((byte) slot)) is
                { ConnectedState: PlayerConnectedState.PlayerConnected } controller)
                EnsureHooked(controller);
        }, 5);
    }

    public void OnClientDisconnected(IGameClient client, NetworkDisconnectionReason reason)
    {
        var slot = client.Slot.AsPrimitive();
        if ((uint) slot >= (uint) Slots)
            return;

        // Evict this slot's rows/cols from every pair matrix so a reused slot never inherits stale state. -1 = no
        // hysteresis latch: this slot is LEAVING, so there is no recovering-oracle re-hide to guard against.
        ResetSlotPairs(slot, -1);
        _snap[slot] = default;
        _weHooked[slot] = false; // controller for this slot is gone (native auto-unhooks) — drop our ownership belief

        // Drop weapons this controller owned. RESET THE NATIVE OWNER LINKAGE FIRST: a dropped weapon (notably a
        // dropped C4) survives the disconnect still owner-linked to this controller INDEX in the transmit manager.
        // Merely dropping our belief would leave that stale native link in place, so when the NEXT player reuses
        // this slot's controller index their hides would drag the orphaned weapon invisible (an invisible ground
        // gun / bomb that follows the new joiner). SetEntityOwner(Invalid) purely un-links → fail-open; a stale
        // weapon index fails gracefully.
        var ctrlIndex = new EntityIndex(new PlayerSlot((byte) slot));
        List<EntityIndex>? gone = null;
        foreach (var kv in _weaponOwner)
        {
            if (kv.Value == ctrlIndex)
                (gone ??= []).Add(kv.Key);
        }
        if (gone is not null)
        {
            foreach (var widx in gone)
            {
                _bridge.TransmitManager.SetEntityOwner(widx, EntityIndex.InvalidIndex);
                _weaponOwner.Remove(widx);
                _weaponUnseenFrames.Remove(widx);
            }
        }
    }

    // ── TEST COMMAND — REMOVE AFTER LIVE 2-CLIENT TRANSMIT TEST (spec §6) ────────────────────────────────
    // Only reachable while fogOfWar.enabled (the listener is installed only then). Chat `fow_test 0|1` forces
    // the transmit state of the FIRST other human (target/sender) for the caller (observer/receiver) on the
    // FOW channel, and SUSPENDS the automatic oracle so the forced state is observable. `0` = hide, `1` = show.
    public ECommandAction OnClientSayCommand(
        IGameClient client, bool teamOnly, bool isCommand, string commandName, string message)
    {
        if (!_installed || string.IsNullOrWhiteSpace(message))
            return ECommandAction.Skipped;

        var parts = message.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        if (parts.Length == 0)
            return ECommandAction.Skipped;

        var cmd = parts[0].TrimStart('!', '/', '.');
        if (!cmd.Equals("fow_test", StringComparison.OrdinalIgnoreCase))
            return ECommandAction.Skipped;

        // ADMIN GATE (silent reject). This module registers no permissions of its own, so we reuse the repo's
        // existing admin mechanism (AdminManager.GetAdmin — same "is a registered admin" test DetectionSink uses
        // for adminBypass) AND require debugStatus. Anyone else's `fow_test` is swallowed with NO feedback: the
        // manual mode suspends the oracle, so an ungated fow_test 0 could grief-freeze every hidden enemy — this
        // must never be reachable by a normal player.
        if (!_debugStatus || _bridge.AdminManager?.GetAdmin(client.SteamId) is null)
            return ECommandAction.Handled;

        if (parts.Length < 2 || !int.TryParse(parts[1], out var arg) || (arg != 0 && arg != 1))
        {
            client.Print(HudPrintChannel.Chat, " [FOW test] usage: fow_test <0=hide|1=show>");
            return ECommandAction.Handled;
        }

        HandleFowTest(client, arg == 1);
        return ECommandAction.Handled;
    }

    private void HandleFowTest(IGameClient caller, bool show)
    {
        if (_bridge.EntityManager.FindPlayerControllerBySlot(caller.Slot) is not
            { ConnectedState: PlayerConnectedState.PlayerConnected } aCtrl)
        {
            caller.Print(HudPrintChannel.Chat, " [FOW test] your controller is not ready");
            return;
        }

        // Pick the first OTHER in-game human as the target (prefer an opposite-team one).
        IPlayerController? bCtrl = null;
        foreach (var other in _bridge.ClientManager.GetGameClients(true))
        {
            if (other.IsFakeClient || other.IsHltv || other.Slot == caller.Slot)
                continue;
            if (_bridge.EntityManager.FindPlayerControllerBySlot(other.Slot) is not
                { ConnectedState: PlayerConnectedState.PlayerConnected } oc)
                continue;

            bCtrl = oc;
            if (oc.Team != aCtrl.Team)
                break; // opposite team preferred
        }

        if (bCtrl is null)
        {
            caller.Print(HudPrintChannel.Chat, " [FOW test] need a second human player");
            return;
        }

        EnsureHooked(aCtrl);
        EnsureHooked(bCtrl);

        if (show)
        {
            // fow_test 1: resume the automatic oracle immediately and re-assert VISIBLE for the forced pair.
            CancelManualTest();
            _bridge.TransmitManager.SetEntityState(bCtrl.Index, aCtrl.Index, transmit: true, _channel);
            caller.Print(HudPrintChannel.Chat, $" [FOW test] SHOW {bCtrl.PlayerName} — oracle resumed");
            return;
        }

        // fow_test 0: suspend the oracle so the forced HIDE stays observable, and ARM a 30s auto-expire so a
        // forgotten test can NEVER freeze a hidden enemy permanently. On expiry (or map change) EndManualTest
        // re-asserts VISIBLE on the pair and the oracle resumes. Release any prior test's pair first.
        CancelManualTest();

        // Entering manual test mode early-returns the ENTIRE OnFrame oracle (see OnFrame), which would otherwise
        // FREEZE every pair currently hidden server-wide — enemies keep moving while invisible for other players
        // until the 30s expiry. Release every believed-hidden pair to plain fail-open visibility FIRST, then apply
        // the single forced hide below, so only the observed pair is culled and the rest of the server degrades to
        // all-visible. Purely adds visibility → fail-open; the single-pair observation is unaffected.
        ForceAllVisible();

        _manualForcedSender   = bCtrl.Index;
        _manualForcedReceiver = aCtrl.Index;
        _manualTestMode       = true;
        // StopOnMapEnd | ForceCallOnStop: on a map change (or an explicit StopTimer during re-arm / Uninstall)
        // the engine force-calls EndManualTest SYNCHRONOUSLY (SharpCore StopTimer/map-end path) so manual mode
        // is always cleared and the oracle resumes — it can never stay suspended into the next map.
        _manualTestTimer = _bridge.ModSharp.PushTimer(
            EndManualTest, 30, GameTimerFlags.StopOnMapEnd | GameTimerFlags.ForceCallOnStop);

        var ok = _bridge.TransmitManager.SetEntityState(bCtrl.Index, aCtrl.Index, transmit: false, _channel);

        caller.Print(HudPrintChannel.Chat,
            $" [FOW test] HIDE {bCtrl.PlayerName} for you — SetEntityState={ok} "
            + "(oracle suspended; auto-resumes in 30s / on map change / plugin reload)");
    }

    // The timer callback. Ends manual test mode and re-asserts VISIBLE on the forced pair so a forced-hidden
    // enemy is never left stuck, letting OnFrame's oracle resume immediately. This is invoked BY the timer
    // (normal 30s expiry) and BY StopTimer's ForceCallOnStop path (map end / re-arm / fow_test 1 / Uninstall) —
    // so it must NOT itself call StopTimer (the timer is still live in the queue during its own callback, which
    // would re-enter). External early-termination goes through CancelManualTest.
    private void EndManualTest()
    {
        _manualTestMode  = false;
        _manualTestTimer = Guid.Empty;

        var sender   = _manualForcedSender;
        var receiver = _manualForcedReceiver;
        _manualForcedSender   = EntityIndex.InvalidIndex;
        _manualForcedReceiver = EntityIndex.InvalidIndex;

        // Fail-open: push VISIBLE on the forced pair. A stale index just fails gracefully (engine default is
        // visible), so this can never leave a hidden enemy stuck.
        if (sender != EntityIndex.InvalidIndex && receiver != EntityIndex.InvalidIndex)
            _bridge.TransmitManager.SetEntityState(sender, receiver, transmit: true, _channel);
    }

    // Early-terminate the manual test (fow_test 1 / re-arm / Uninstall). Cancelling the timer force-calls
    // EndManualTest synchronously (ForceCallOnStop); when no timer is armed we invoke it directly so the state
    // is still cleared. Runs EndManualTest exactly once.
    private void CancelManualTest()
    {
        if (_manualTestTimer != Guid.Empty)
            _bridge.ModSharp.StopTimer(_manualTestTimer);
        else
            EndManualTest();
    }

    // ── IEntityListener (game thread) ─────────────────────────────────────────────────────────────────────
    // Evict a deleted weapon's tracking immediately so a recycled entity index can never make the weapon
    // gone-path (ReconcileWeapons) push SetEntityOwner(widx, Invalid) onto a FOREIGN hook. Fires for every
    // entity deletion, so it stays cheap (three hash removes on a usually-absent key). Native auto-unhooks the
    // deleted entity, so no RemoveEntityHooks is needed here.
    public void OnEntityDeleted(IBaseEntity entity)
    {
        if (!_installed)
            return;

        var idx = entity.Index;
        _weaponOwner.Remove(idx);
        _weaponSeen.Remove(idx);
        _weaponUnseenFrames.Remove(idx);
        _weHookedWeapons.Remove(idx);
    }
}
