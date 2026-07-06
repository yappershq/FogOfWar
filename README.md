<div align="center">
  <h1><strong>FogOfWar</strong></h1>
  <p>Anti-wallhack transmit-denial for CS2 / ModSharp — hides enemies your client provably can't see, so a wallhack/ESP has no pawn state to read.</p>
</div>

<p align="center">
  <a href="https://github.com/Kxnrl/modsharp-public"><img src="https://img.shields.io/badge/framework-ModSharp-5865F2?logo=github" alt="ModSharp"></a>
  <img src="https://img.shields.io/badge/game-CS2-orange" alt="CS2">
  <img src="https://img.shields.io/github/stars/yappershq/FogOfWar?style=flat&logo=github" alt="Stars">
</p>

---

**FogOfWar** stops the server from networking an enemy pawn to a client that provably cannot see it —
not a detector, it never warns/kicks/bans. On map load it bakes the map's static `world_physics`
collision into an immutable BVH (opacity-filtered so fences/grates/glass don't occlude), then a
background worker thread continuously recomputes the full enemy-visibility matrix against it. The
game thread only reads the latest snapshot and denies transmit for pawns that are blocked, under a
strict fail-open rule — any degradation (stale snapshot, worker not ready, missing pair) always errs
**visible**, never hidden.

Concept ported from [karola3vax/CS2FOW](https://github.com/karola3vax/CS2FOW) (MIT); this is a
from-scratch C# re-implementation for ModSharp (BVH bake + off-thread worker + `ITransmitManager`
denial instead of the original's approach).

## 🚀 Install

Copy the build output into your ModSharp install (`<sharp>` = your `sharp` directory):

| From | To |
|------|----|
| `.build/modules/FogOfWar.Core/` | `<sharp>/modules/FogOfWar.Core/` |

Restart the server (or change map) to load. `configs/fogofwar.json` is auto-generated on first run
with `enabled: false` — the module is completely inert until you opt in.

## 🧩 Dependencies

Uses the **ModSharp first-party module** (ships with ModSharp): **AdminManager**, resolved optionally
— only gates the `fow_test` debug command (see below). Everything else (transmit hooks, entity/client
listeners, the frame hook) is base ModSharp.

Bundled: `ValveResourceFormat` (Source2Viewer's VRF library, native runtimes trimmed) — reads the
map's `.vpk` to extract `world_physics` collision at bake time. Ships inside the module (~4 MB).

## ⌨️ Commands

| Command | Description | Permission |
|---------|-------------|------------|
| `fow_test <0\|1>` | Debug-only: force-hide (`0`) or force-show (`1`) the caller's LOS vs. the first other human, suspending the automatic oracle for 30s so the forced state is observable. | Registered admin (via AdminManager) **and** `debugStatus: true` in config — silently ignored otherwise. |

## ⚙️ Configuration

`configs/fogofwar.json` (auto-generated on first run):

| Key | Default | Meaning |
|-----|---------|---------|
| `enabled` | `false` | Master switch. Module is fully inert (no hooks, no bake, no worker) until set `true`. |
| `updateIntervalMs` | `15` | Background-worker recompute cadence (ms) for the full visibility matrix. |
| `minLookaheadMs` | `180` | Lower clamp on the peek-prediction lookahead window. |
| `maxLookaheadMs` | `250` | Upper clamp on the peek-prediction lookahead window. |
| `assumedRttMs` | `60` | Stand-in for per-client RTT (ModSharp exposes no per-client latency); feeds the lookahead calc. |
| `peekMarginUnits` | `21` | Extra margin (units) added to predicted travel distance so a corner-peeker reveals slightly early. |
| `visibilityHoldMs` | `150` | Hysteresis hold: once a pair is seen visible it stays visible at least this long (anti-flicker). |
| `maxPredictionSpeed` | `500` | Velocity cap (units/s) used for the lookahead offset — caps a boosted/exploited speed. |
| `staleVisibleMs` | `250` | Fail-open threshold: if the newest worker snapshot is older than this, every pair is forced visible. Auto-raised at install if too small vs. `updateIntervalMs`. |
| `hideWeapons` | `true` | Also hide weapons a hidden pawn carries (native transmit-owner mechanism, ~zero extra cost). |
| `transmitChannel` | `3` | Dedicated `ITransmitManager` channel (0–5) so FOW composes (AND) with other transmit plugins. |
| `debugStatus` | `false` | Emit a rolling stats log line every 60s + enable the `fow_test` command + verbose bake logging. |
| `extraSeeThroughProps` | `[]` | Extra surfaceprop names to treat as see-through (non-occluding), on top of the built-in set (chain-link/grates/glass). Add a false-hiding custom-map surfaceprop here (the bake log names the top included props). |

## 🔧 How it works

Line-of-sight is never traced live: at map start the static world collision is baked once into an
immutable BVH (CRC-cached to disk so a repeat map load skips the extraction), and a dedicated worker
thread recomputes the whole alive-enemy visibility matrix every `updateIntervalMs` against that BVH —
2(+1 vertical peek) observer points × 8 target points per pair, with peek-prediction lookahead. The
game thread only ever reads the latest published snapshot (volatile swap) and calls
`ITransmitManager.SetEntityState` to cull a blocked pawn (and its weapons) for that one receiver;
hidden pawns' **controllers** are never touched, so scoreboard/team data stay intact. Every failure
mode — worker not started, stale snapshot, bake not ready, an entity missing its hook — resolves to
**visible**, so a bug can only under-hide, never falsely hide a genuinely visible enemy.

## 📦 Build

```bash
dotnet build FogOfWar.slnx -c Release
```

Outputs `.build/modules/FogOfWar.Core/FogOfWar.dll` (~4 MB with the bundled VRF library after native
runtime trimming).

## 🙏 Credits

Concept ported from [karola3vax/CS2FOW](https://github.com/karola3vax/CS2FOW), MIT License. The
transmit-denial approach, the peek/prediction model (lookahead, peek margin, visibility hold,
`k_max_prediction_speed`), and the config defaults follow upstream; the BVH bake + off-thread worker
+ ModSharp `ITransmitManager` integration is a from-scratch C# implementation.

---

<div align="center">
  <p>Made with ❤️ by <a href="https://github.com/yappershq">yappershq</a></p>
  <p>⭐ Star this repo if you find it useful!</p>
</div>
