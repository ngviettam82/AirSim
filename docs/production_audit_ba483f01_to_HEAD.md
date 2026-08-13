# Production Audit Report

**Range:** `ba483f010044190f8f8cf590ced96a67d4682105` → `cff11d4c7cf004e714103d0947febbb7d15a3117` (HEAD)  
**Span:** 52 commits · ~595 files changed  
**Date:** 2026-08-13  
**Repo:** `ngviettam82/AirSim` (fork of Cosys-Lab AirSim / UE 5.5)  
**Method:** Multi-agent deep dive (read-only): segmentation/stencil, physics/battery, GPU LiDAR, CameraHost/equirect, ROS2/recording, Python/CI, spawn/annotation, chronology, RPC surface, Unreal recording, docs fidelity.

---

## 1. Executive summary

This range turns a Cosys-Lab UE 5.5 AirSim tree into a **field-oriented fork** with three stacked product arcs:

1. **Perception** — source CustomStencil Seg/IR, equirectangular capture, CameraHost + gimbal, UE 5.5 stencil view extension, subwindow capture fix  
2. **Data / control** — free-run recording, MCAP, PX4 ROS 2 live control, multirotor plant physics + battery telemetry  
3. **Productization** — Multirotor GPU LiDAR, Python package `airsim`, GitHub Actions CI, manual plugin packaging for `v3.4.0`

| Metric | Assessment |
|--------|------------|
| **Overall production readiness** | **~6.8 / 10** |
| **Architecture quality** | Strong on Seg/IR stencil, CameraHost bounds, recording provenance, battery MAVLink identity |
| **Main residual theme** | Oversell vs code (custom annotation), thin CI for Unreal/AirLib, LAN unauthenticated surfaces, Multirotor LiDAR async tradeoffs |

**Ship-ready for (with docs/ops discipline):**

- UE 5.5 Blocks/EVN-style simulation  
- Built-in Segmentation / Infrared via source stencil  
- CameraHost on loopback  
- Python client installed **from this repository**  
- Multirotor physics + PX4 battery SITL (configured correctly)  
- Multirotor GPU LiDAR with known rate/async limits  

**Not fully production-ready for:**

- “Full Cosys multi-layer custom annotation” (hard-disabled)  
- Untrusted LAN (open RPC / CameraHost)  
- “CI proves the product” (no UE/AirLib/RPC matrix)  
- Claiming every tag auto-ships a verified plugin zip (manual process)

---

## 2. Chronology (52 commits)

### 2.1 Theme counts (approximate)

| Theme | Count |
|-------|------:|
| Feature | ~28 |
| Fix | ~15 |
| Docs | ~5 |
| CI | ~6 |
| Merge | ~2 |
| Revert | **1** (`47965cbc`) |

### 2.2 Full timeline (oldest → newest)

| # | SHA | Subject | Theme |
|---|-----|---------|-------|
| 0 | `ba483f01` | Merge branch `5.5dev` (base) | merge |
| 1 | `cf34b2eb` | Configure Git LFS for large media | ci |
| 2 | `c694e69d` | Fix segmentation camera annotation alignment | feature |
| 3 | `2c63a94d` | Add infrared annotation support | feature |
| 4 | `b691e662` | Instance segmentation + IR (PIPCamera/SimMode) | feature |
| 5 | `c958f54b` | Refactor annotation material loading | fix |
| 6 | `0814690e` | Docs: image APIs / instance segmentation | docs |
| 7 | `cef00c99` | Landscape support in segmentation/IR | feature |
| 8 | `bfb1cfe6` | Merge `fix/segment` into main | merge |
| 9 | `b8f1ee69` | Add CI/CD workflows | ci |
| 10 | `f706e072` | Add GitLab CI/CD pipeline | ci |
| 11 | `df7852c2` | Add batch segmentation ID API | feature |
| 12 | `e770413e` | Implement source stencil segmentation backend | feature |
| 13 | `d008a876` | Update source stencil annotation docs | docs |
| 14 | `9be61c9a` | Fix source stencil annotation audit issues | fix |
| 15 | `d67d2136` | Fix annotation backend stencil conflicts | fix |
| 16 | `893ec24a` | Optimize source stencil segmentation paths | fix |
| 17 | `5bb05cdf` | Clarify source stencil review build verification | docs |
| 18 | `1cb82e14` | Remove tutorial LFS assets | fix |
| 19 | `2bcfd6ae` | Remove stale tutorial media reference | fix |
| 20 | `09958394` | Normalize repository line endings | fix |
| 21 | `d126ae8f` | Add equirectangular image projection support | feature |
| 22 | `960deb80` | Equirectangular sampling map / color sampling | feature |
| 23 | `d38c68a1` | update docs | docs |
| 24 | `cc36a5e9` | Finalize equirectangular capture support | feature |
| 25 | `4d802e9d` | Add GPU equirectangular subwindow preview | feature |
| 26 | `5ab46043` | Keep shader plugin in project workflows | fix |
| 27 | `483c8ea1` | Fix HUD subwindow aspect workflow | fix |
| 28 | `0853dd06` | Add native settings-driven CameraHost streaming | feature |
| 29 | `23a37e7b` | Build script VS checks for UE 5.5 | fix |
| 30 | `8eaf75e9` | “Fix stable segmentation and infrared labels” | feature ⚠️ |
| 31 | `ea3723a6` | Add CameraHost gimbal control API | feature |
| 32 | **`47965cbc`** | **Revert bad segmentation readback path** | **revert** |
| 33 | `c767b2ef` | UE 5.5 stencil + GPU view extension | feature |
| 34 | `4f73f774` | Fix proxy-free UE 5.5 stencil capture | fix |
| 35 | `9515657f` | Physics-paused settings-driven sensor recording | feature |
| 36 | `09f22e83` | Ignore recording sessions; MSVC toolset for CI | fix/ci |
| 37 | `10bad888` | Timestamped free-run recording synchronization | feature |
| 38 | `16867e61` | ROS 2 MCAP recording | feature |
| 39 | `f6ee6ae5` | Synchronized PX4 ROS 2 live control | feature |
| 40 | `2212f3f9` | Multirotor Physics + PX4 battery telemetry | feature |
| 41 | `7202d829` | Rebrand docs to AirSim / this repo | docs |
| 42 | `7839adb0` | Fix PX4 external battery acceptance (failsafe) | fix |
| 43 | `5a3b7671` | Per-vehicle MAV_SYS_ID for battery telemetry | fix |
| 44 | `53f1a09e` | Port Cosys UE 5.5 realism / annotation fixes | feature ⚠️ |
| 45 | `46098339` | Rename Python package cosysairsim → airsim | feature |
| 46 | `cc851836` | Harden spawn / async GPU LiDAR / Physics arms | fix |
| 47 | `4bda1a5b` | Fix Seg/IR subwindows black after stencil configure | fix |
| 48 | `376f2fdf` | Replace GitLab + legacy GH CI with GHA pipeline | ci |
| 49 | `5712f3ed` | Close pipeline gaps (nightly, Dependabot, docs) | ci |
| 50 | `f5b55326` | Merge PR #1 CI pipeline | merge |
| 51 | `dd6c5818` | Fix CI/release wheel smoke to import airsim | fix |
| 52 | `cff11d4c` | Fix BuildPlugin: drop IsNaniteEnabled log | fix |

### 2.3 Narrative arcs

```text
ba483f01  Cosys UE 5.5 base
    │
    ├─ Arc 1 Perception ─────────────────────────────────────────┐
    │  proxy Seg/IR → source stencil → equirect → CameraHost     │
    │  bad shared readback (8eaf75e9) → REVERT (47965cbc)        │
    │  view extension (c767b2ef) → black HUD fix (4bda1a5b)      │
    │                                                             │
    ├─ Arc 2 Data/control ───────────────────────────────────────┤
    │  recording free-run → MCAP → PX4 ROS2 → physics + battery  │
    │                                                             │
    └─ Arc 3 Productization ─────────────────────────────────────┘
       Cosys Multirotor LiDAR → airsim package → GHA → package fix
       HEAD cff11d4c
```

---

## 3. Critical integrity events

| Event | Status at HEAD |
|-------|----------------|
| Shared Seg→IR readback (`8eaf75e9`) | **Fully cleaned** by `47965cbc`; rebuilt via view extension |
| Black Seg/IR subwindows (`c767b2ef` forced everyframe false) | **Fixed** by `4bda1a5b` |
| CI broken after package rename | **Fixed** by `dd6c5818` |
| BuildPlugin `IsNaniteEnabled` (editor-only API) | **Fixed** by `cff11d4c` — log only, no spawn/Nanite behavior change |
| Cosys custom annotation “ported” | **Incomplete** — proxy create still hard-disabled |
| Dual CI (GitLab vs GHA) | **Superseded** by GitHub Actions |

### 3.1 Revert detail (`47965cbc`)

`8eaf75e9` introduced a **SharedLabelReadback** path that could redirect Segmentation capture toward Infrared and post-colorize — high risk for wrong labels. The revert removed that machinery. The production path today is **source CustomStencil + `AirSimStencilViewExtension`** (2D) and materials on equirect cubes — not a restore of the pre-`8eaf` proxy mirror system for built-in Seg/IR.

---

## 4. Subsystem assessments

### 4.1 Source-stencil Segmentation / Infrared

**Score: 7.5 / 10**

**What works**

- Built-in Seg/IR use source CustomStencil (no ISM/HISM mirror explosion).  
- IDs clamped 0..255; shared annotator for Seg + IR.  
- ShowFlags tuned for UE 5.5 CustomDepth reliability.  
- Subwindow continuous capture: `setCameraTypeUpdate` owns `bCaptureEveryFrame`; `ConfigureSourceStencilCaptureState` no longer forces false.  
- API path: game-thread `CaptureScene` + readback with provenance.  
- Equirect cubes: PP materials + 2-frame IR/Seg warmup.  
- `r.CustomDepth=3` at HUD startup.

**Risks**

| Sev | Issue |
|-----|--------|
| P0 | View extension: null CustomDepth returns **tonemapped scene color**, not black/labels (`AirSimStencilViewExtension`) |
| P1 | Dual label backends (view extension vs content materials vs Lidar) can drift |
| P1 | View extension `CreateRaw` lifetime vs EndPlay |
| P1 | No automated visual regression (subwindow non-black, ID roundtrip) |
| P2 | 8-bit ID collisions after 255 distinct labels |
| P2 | CustomDepth shared with GPU LiDAR intensity |

**Verdict:** Architecture sound; black-subwindow regression closed in code; fail-open on missing CustomDepth is the remaining perception P0.

---

### 4.2 Custom annotation layers (proxy)

**Score: 3.5 / 10**

- `ASimModeBase::InitializeAnnotation` **ignores** all custom `Annotation[]` layers (stencil-only build).  
- `FObjectAnnotator::CanCreateProxyAnnotationComponent` always returns **false**.  
- Large Cosys Nanite/proxy code still present but largely **unreachable**.  
- Settings still parse `Backend` / `ProxyComponentBudget`.  
- Docs/CHANGELOG still describe multi-layer proxy annotation.

**Verdict:** **Product honesty gap** — either re-enable budgeted proxy or rewrite docs to “Seg/IR source stencil only.”

---

### 4.3 Multirotor physics + PX4 battery

**Score: 7.0–7.5 / 10**

**Pipeline**

```text
settings Vehicles.*.Physics
  → MultirotorPhysicsConfig::fromSettingsJson
  → MultiRotorParams::applyPhysicsConfig (after frame setup)
  → MultiRotorPhysicsBody (rotors, battery_)
  → FastPhysicsEngine (ang drag, wind turb)
  → MavLinkMultirotorApi::updateBatteryTelemetry
  → BATTERY_STATUS ~5 Hz on control link
```

**Correct**

- Sysid = vehicle target heartbeat (`5a3b7671`).  
- Compid = 191 companion (`7839adb0`) — not autopilot / GCS-like identity.  
- `ArmLength` expands to rotor count (quad/hex/octo); optional `ArmLengths`.  
- Unit tests cover core math, drain, GE edges, arm expansion logic.  
- Docs: `docs/multirotor_physics.md` high quality.

**Risks**

| Sev | Issue |
|-----|--------|
| P1 | Default **EnableGroundEffect / EnableThrustAirSpeed true** without `Physics` block — silent plant change |
| P1 | Wrong-size `ArmLengths` → silent no-op; can block `ArmLength` |
| P1 | Blacksheep + `ArmLength` rebuilds as symmetric QuadX |
| P1 | Empty plant battery still ~62% thrust scale; failsafe is PX4-side |
| P1 | `SIM_BAT_ENABLE` must be 0 for external battery — ops only, not enforced |
| P2 | Ground AGL from `-z` only; axial flow symmetric climb/descent |

**Verdict:** Coherent SITL feature set; not fully hardened against silent JSON mistakes.

---

### 4.4 Multirotor GPU LiDAR (Cosys async)

**Score: 7.0 / 10**

**What works**

- Multirotor + GPU LiDAR ban removed.  
- `async_capture_mode` when SimMode is Multirotor.  
- CaptureScene / ReadPixels only on game thread in async path.  
- FOV balloon fix: do not integrate rotation while capture in flight.  
- Mutex on pixel buffers; empty depth does not mark ready.  
- Unit test Multirotor async vs Car sync.

**Risks**

| Sev | Issue |
|-----|--------|
| P0 | Residual off-GT UE access: actor pose, weather scalar, optional debug draw from physics path |
| P0/P1 | Empty depth → sector angles already advanced → silent holes |
| P1 | Effective scan rate gated by render FPS, not full `RotationsPerSecond` |
| P1 | `UpdateFrequency` / startup delay not applied like classic Lidar |
| P1 | `GPULidarData` output unsynchronized vs concurrent RPC |
| P1 | EndPlay vs physics race |

**Verdict:** Directionally productionized; document async rate and harden GT hygiene before calling it field-proven.

---

### 4.5 CameraHost + gimbal + equirect

**Score: 7.0–7.5 / 10**

**CameraHost validation (strong)**

- Method allowlist, path allowlist, header 16 KiB, body 64 KiB.  
- Structured JSON errors.  
- Max connections.  
- Gimbal: finite ranges, text/plain 6 fields, game-thread apply.  
- **Gimbal relative rotation only; mount location re-pinned** (Agents.md compliant).  
- Hosted cameras start with `nodisplay=true` (no idle continuous capture).  
- MJPEG/snapshot stay JPEG.

**Risks**

| Sev | Issue |
|-----|--------|
| P0 | No auth/TLS; `0.0.0.0` + open port = unauth video + **gimbal as actuator** |
| P0 | CORS `*` |
| P1 | Visible subwindow + host → double capture |
| P1 | Equirect continuous host streams are expensive |
| P1 | Global image capture mutex shared with RPC/recording |
| P2 | Sampling map cache growth; IPv4 only |

**Equirect:** mode on existing image types; 2:1 layout; Scene seam mitigation; Vulkan cube readback rejected; label cube warmup present.

**Verdict:** Production-minded sim host; security is bind/firewall discipline, not code auth.

---

### 4.6 Recording + MCAP

**Score: 7.0–7.5 / 10**

**Model at HEAD:** free-run dual-rate (not “physics frozen for shutter”).

| Field | Provenance |
|-------|------------|
| FrameTimeStamp | Latch under short physics mutex |
| ImageTimeStamp | Capture-state stamp before CaptureScene; MCAP omits unproven |
| Sensors | Native stamps; IMU history ring for Rosbag |

**Risks**

| Sev | Issue |
|-----|--------|
| P1 | Global image mutex can stall concurrent image APIs; GT re-entry hazard |
| P1 | ScalableClock advances under physics pause while recording continues |
| P1 | GPS/Baro/Mag can undersample vs IMU |
| P1 | Dual bags (Unreal MCAP + PX4) need offline merge |
| P2 | Unchunked MCAP; `isRecording()` false before bag finalize |

**Verdict:** Strong for stated free-run model; do not claim hard multi-sensor shutter sync.

---

### 4.7 PX4 ROS 2 live control

**Score: 6.5 / 10**

**Strengths**

- Control ownership isolation (`ControlMode: PX4`).  
- Exact `px4_msgs` schema validation (`px4_schema.py`).  
- HIL membership proof for image pairing.  
- Frame gate; failsafe fencing; solid pure-Python unit tests.  
- Rate bridge does not arm / set OFFBOARD.

**Gaps**

| Sev | Issue |
|-----|--------|
| P0 | Ops: `UXRCE_DDS_SYNCT` must be 0; wrong default breaks binding |
| P0 | CI does **not** build `airsim_ros_pkgs` (C++ wrapper) |
| P0 | No SITL/uXRCE integration CI |
| P1 | HIL history published 512 vs AirLib 2048 capacity |
| P1 | `world` vs `world_enu` if not using PX4 wrapper launch |
| P1 | px4_msgs pin is a historical commit, not “PX4 1.16 release” |

**Verdict:** Design is production-minded; automation coverage is thin for the full stack.

---

### 4.8 WorldSimApi spawn + Nanite log

**Score: 8 / 10 (spawn path)**

| Check | Result |
|-------|--------|
| Game-thread load only | Pass |
| `FSoftObjectPath::TryLoad` | Pass |
| No shared `NaniteSettings` mutation | Pass |
| Unique name without open regex | Pass (map-only residual) |
| `IsNaniteEnabled` log removal | Correct: API is `#if WITH_EDITORONLY_DATA` — fails **BuildPlugin**, not “UE 5.8 only” |

**Note:** `IsNaniteEnabled` **exists on UE 5.5**, but only under `WITH_EDITORONLY_DATA`. Editor/Blocks builds can compile it; RunUAT package targets cannot. Removal does **not** change spawn/Nanite rendering behavior.

---

### 4.9 Python client + CI + release

**Score: 7.5–8.0 (client) · 7.5 (CI design) · 6.5 (plugin packaging process)**

**Python**

- Package name `airsim`; install from repo only.  
- No leftover `import cosysairsim` in code.  
- CI/release smoke: `import airsim` (post-`dd6c5818`).  
- Residual risk: PyPI name collision with unrelated `airsim` wheels.

**CI covers**

- Text hygiene, workflow YAML shape, mkdocs strict  
- Wheel build + import  
- ROS 2 Humble + `airsim_px4_offboard` tests  

**CI does not cover**

- AirLib C++ / unit tests  
- Unreal plugin compile on PR  
- CameraHost / Seg visual / LiDAR races  
- RPC round-trip  

**Release**

- `release.yml` (hosted): Python + ROS image + GitHub Release  
- `unreal-package.yml` (self-hosted): optional; **no runner currently** → manual zip  
- `v3.4.0` includes `AirSimPlugin-Win64.zip` (manual BuildPlugin attach)

---

### 4.10 RPC / network surface

**CameraHost:** largely meets Agents.md validation bar.  
**RpcLib:** trusted-LAN design; uneven validation.

| P0 RPC risks | Notes |
|--------------|-------|
| `simRunConsoleCommand` | Unauth UE console |
| `simCreateVoxelGrid` | Unbounded resize |
| Default `EnableRpc: true` | Full vehicle/world control if reachable |

---

### 4.11 Documentation fidelity

**Good:** `multirotor_physics.md`, `camera_host.md`, `ci_cd.md`, instance segmentation default false, primary Python install warnings.

**Critical doc bugs**

| Issue | Severity |
|-------|----------|
| Docs use `RpcEnabled`; code reads **`EnableRpc`** | P0 |
| `settings.md` defaults dump: EngineSound/Lumen wrong | P0 |
| mkdocs/README still Cosys branding / cosys-lab.github.io | P0–P1 |
| `multirotor_physics.md` not in mkdocs nav | P1 |
| packaging.md incomplete vs dual-plugin BuildPlugin | P1 |
| install_precompiled under-spec’d | P1 |
| GPU LiDAR example still SkidVehicle-first | P2 |
| Client version 3.3.0 vs product 3.4 | P1 |

---

## 5. Production readiness scores (summary)

| Subsystem | Score | Notes |
|-----------|------:|-------|
| Source-stencil Seg/IR | 7.5 | Architecture + HUD fix; fail-open CustomDepth |
| Custom annotation proxy | 3.5 | Hard-disabled; docs oversell |
| Equirect | 7.0 | Solid; expensive under continuous host |
| CameraHost + gimbal | 7.5 | Strong bounds; bind = security boundary |
| GPU LiDAR Multirotor | 7.0 | Async correct direction; GT/rate residual |
| Multirotor Physics | 7.0 | Documented; silent arm misconfig |
| PX4 battery telemetry | 7.5 | Identity fixed; ops-coupled |
| Recording free-run / MCAP | 7.5 | Proven stamps; free-run model |
| PX4 ROS 2 live control | 6.5 | Design mature; CI thin |
| Python `airsim` | 8.0 | Rename solid; PyPI collision |
| CI/CD | 7.5 | Honest thin gates |
| Plugin packaging process | 6.5 | Manual; dual-plugin required |
| Docs / rebrand | 6.0–7.0 | Physics good; settings/rebrand gaps |
| **Overall** | **6.8** | Field fork with known ops/docs debts |

---

## 6. Risk register (prioritized)

### P0 — ship blockers or field footguns

1. Custom annotation advertised but **disabled** — docs/product honesty.  
2. Stencil view extension **fail-open** (scene color when CustomDepth missing).  
3. Settings docs: **`EnableRpc`** not `RpcEnabled`; fix defaults dump (EngineSound, Lumen).  
4. Network: open RPC / CameraHost on non-loopback = full sim control + video + gimbal.  
5. GPU LiDAR Multirotor: off-GT UE access residual; empty-depth sector loss.  
6. Plugin release: no self-hosted runner — manual zip needs tag/SHA discipline.  
7. ROS2 ops: `UXRCE_DDS_SYNCT=0`; CI skips C++ wrapper.

### P1 — quality / silent failure

8. Physics defaults (GE/axial) change plant without `Physics` block.  
9. Silent `ArmLengths` mismatch / Blacksheep arm rebuild.  
10. Battery plant empty ≠ no lift; dual battery models if PX4 sim battery left on.  
11. Dual Seg label backends drift.  
12. View extension lifetime (`CreateRaw`).  
13. Recording mutex + ScalableClock under pause.  
14. Incomplete Cosys rebrand; version skew 3.3.0 vs 3.4.  
15. Spawn uniqueness only vs `scene_object_map`.  
16. packaging.md vs real BuildPlugin (AirSimShaders first).

### P2 — polish

17. Label capture always-persist cost.  
18. Physics AGL / axial symmetry.  
19. MCAP unchunked.  
20. Dead Cosys proxy code surface.  
21. mkdocs nav for physics; multirotor GPU LiDAR examples.  
22. Stronger Python smoke (client classes + version).

---

## 7. What is correct (keep)

- Source CustomStencil for built-in Seg/IR (EVN-scale intent).  
- Subwindow capture rate ownership after `4bda1a5b`.  
- Spawn game-thread TryLoad + no shared Nanite mutation.  
- `cff11d4c` packaging log fix (editor-only API).  
- Battery MAVLink sysid/compid fixes.  
- Multirotor GPU LiDAR: GT capture, FOV integrate pause, buffer mutex.  
- CameraHost bounds + gimbal mount pin.  
- Recording frame-proven image stamps.  
- Python rename + CI import fix.  
- Revert of bad shared Seg/IR readback.

---

## 8. Recommended production polish order

1. **Honesty pass** — stencil-only annotation docs; settings key/defaults; rebrand entry points.  
2. **Perception** — CustomDepth null fail-closed; optional visual smoke.  
3. **LiDAR Multirotor** — GT pose cache; empty-depth reschedule; document rate.  
4. **Physics ops** — arm mismatch logs; default GE migration note.  
5. **Network checklist** — bind warnings; never public RPC without firewall.  
6. **Release** — tag ↔ zip SHA; packaging.md = dual-plugin BuildPlugin steps.  
7. **CI growth (optional)** — AirLib unit job; self-hosted plugin only if permanent runner returns.

---

## 9. Suggested validation checklist (ops)

### Perception

- [ ] Seg + IR subwindow live (non-black) after cold start  
- [ ] `simGetImages` Seg/IR ID roundtrip on known mesh  
- [ ] Equirect Seg first call after warmup  
- [ ] Toggle subwindow off/on after object ID change  

### Physics / PX4

- [ ] Hex/octo `ArmLength` radii  
- [ ] Battery `EnableBattery` + `SIM_BAT_ENABLE=0` + failsafe params  
- [ ] Multi-vehicle sysid isolation  

### GPU LiDAR Multirotor

- [ ] Full 360 cloud cadence vs RPS  
- [ ] Empty/heavy scene no FOV balloon  
- [ ] Intensity + segmentation both on → stencil warning  

### CameraHost

- [ ] Bind `127.0.0.1` by default  
- [ ] Gimbal reset pins mount location  
- [ ] Hosted Seg stream with subwindow hidden  

### Release

- [ ] `import airsim` from release wheel  
- [ ] Plugin zip: `Plugins/AirSim` + `Plugins/AirSimShaders` under UE 5.5  

---

## 10. Release snapshot (context)

| Item | State |
|------|--------|
| Tag | `v3.4.0` |
| Release | https://github.com/ngviettam82/AirSim/releases/tag/v3.4.0 |
| Plugin asset | `AirSimPlugin-Win64.zip` (manual BuildPlugin; Intermediate/PDBs stripped) |
| Python | `airsim-3.3.0` wheel/sdist on same release |
| Self-hosted Unreal runner | **Removed** — package is manual |

Tag `v3.4.0` points at `dd6c5818` (CI import fix). Subsequent `main` includes `cff11d4c` (BuildPlugin log fix); the published zip includes that packaging fix.

---

## 11. Bottom line

This 52-commit line delivers a **coherent, production-minded AirSim fork**: source-stencil perception at scale, CameraHost, recording/MCAP, PX4 ROS 2 control, multirotor plant + battery, Multirotor GPU LiDAR, modern GHA CI, and a documented (if thin) release path.

**Most functional work is directionally correct.** Remaining production debt is concentrated in:

1. **Honesty** (custom annotation off but still described),  
2. **Ops footguns** (settings keys, physics defaults, LAN bind, PX4 battery params),  
3. **Thin automated proof** (CI does not run Unreal/AirLib),  
4. **Hardening edges** (CustomDepth fail-open, LiDAR async rate/empty sectors).

Treat green CI as **necessary but not sufficient**. Field readiness depends on the ops checklist above plus honest documentation of stencil-only annotation and free-run/async limitations.

---

## 12. Document control

| Field | Value |
|-------|--------|
| Report path | `docs/production_audit_ba483f01_to_HEAD.md` |
| Audit type | Read-only multi-agent deep dive |
| Code changes from audit | None |
| Follow-up | Implement polish tracks only after explicit product prioritization |

---

*End of report.*
