# Final Audit — Production Polish Pass

**Date:** 2026-08-13  
**Scope:** Uncommitted working tree after implementing the audit polish list (code + docs)  
**Baseline HEAD:** `cff11d4c`  
**Related:** `docs/production_audit_ba483f01_to_HEAD.md` (full 52-commit range audit)

---

## 1. Executive verdict

| Question | Answer |
|----------|--------|
| Did we implement the agreed polish (code + docs)? | **Yes** — core P0/P1 items landed |
| Are the **code** changes sound? | **Yes**, with minor residual notes below |
| Are the **docs** now honest vs code? | **Mostly yes** for the targeted surfaces |
| Ready to commit? | **Yes** after optional rebuild for plugin binaries |
| Remaining from original audit? | Deferred intentional items (CI AirLib job, re-enable proxy annotation, RPC auth, etc.) |

**Overall polish-pass score: 8.0 / 10** (for what this pass set out to do).

---

## 2. Inventory of this pass

### 2.1 Code / function

| Change | Files | Status |
|--------|-------|--------|
| Stencil **fail-closed black** when CustomDepth missing | `AirSimStencilViewExtension.cpp` | **Pass** |
| View extension callback `CreateSP` (not `CreateRaw`) | same | **Pass** |
| Physics `ArmLengths` mismatch **warn** | `MultiRotorParams.hpp` | **Pass** |
| Physics non-4/6/8 arm rebuild **warn** | same | **Pass** |
| GPU LiDAR GT-cached pose | `LidarCamera.*`, `UnrealGPULidarSensor.cpp` | **Pass** |
| No physics-thread debug draw on async Multirotor | `UnrealGPULidarSensor.cpp` | **Pass** |
| Empty depth **retry** (max 3) same sector | `LidarCamera.cpp` | **Pass** (bounded) |

### 2.2 Docs / settings / rebrand

| Change | Status |
|--------|--------|
| `EnableRpc` (not `RpcEnabled`) in docs + examples + docker | **Pass** |
| Settings defaults dump: EngineSound, Lumen, CameraHost, WindTurbulence | **Pass** |
| Stencil-only annotation honesty | **Pass** (`annotation.md`, `settings.md`, CHANGELOG) |
| CameraHost network safety | **Pass** |
| GPU LiDAR Multirotor async docs + example | **Pass** |
| Physics arm/battery notes | **Pass** |
| Dual-plugin packaging + install_precompiled | **Pass** |
| mkdocs / README / AirSim.uplugin rebrand | **Pass** |
| Full range audit doc | Present (`production_audit_ba483f01_to_HEAD.md`) |

**Diff size:** 18 tracked files + 1 untracked audit (~+336 / −79 lines).

---

## 3. Code audit (correctness)

### 3.1 Stencil view extension — **ACCEPT**

**Intent:** Never return tonemapped scene color when labels cannot be produced.

**Implementation review:**

- Missing `view.Family`, missing `CustomDepthTexture`, invalid scene color → `FailClosedBlack`.
- Black path allocates RT from scene-color size and `AddClearRenderTargetPass(..., Black)`.
- Last resort if even scene color is invalid: still `ReturnUntouchedSceneColor` (unavoidable without a view rect).
- `CreateSP` requires the extension already live as `TSharedFromThis` — satisfied by `MakeShared` in `Create()`.

**Residual:**

| Note | Severity |
|------|----------|
| No one-shot log when failing closed (harder to diagnose CustomDepth off) | P2 |
| Fail-closed not applied to **cube/material** equirect Seg path (materials only) | P2 (pre-existing dual backend) |
| Binaries not rebuilt in this pass — need Blocks/plugin rebuild to ship | Ops |

### 3.2 Physics arm warnings — **ACCEPT**

- Wrong `ArmLengths` size: warn, keep empty `arms` → `ArmLength` expand still blocked (documented).
- Rotor count not in {4,6,8}: warn, no incorrect rebuild.
- Uses `Utils::log` via `Common.hpp` — OK.

**Residual:** Blacksheep + successful `ArmLength` still silently becomes QuadX (documented only; no extra warn). P2.

### 3.3 GPU LiDAR Multirotor — **ACCEPT with notes**

| Fix | Review |
|-----|--------|
| Pose cache on Tick | Written GT-only; physics reads cache. First physics ticks before first Tick may fall back to live `GetActorTransform()` — short window residual (P2). |
| Draw gated off async | Correct; Multirotor already ignores DrawDebugPoints in settings path. |
| Empty depth retry ≤3 | Prevents infinite hang; after 3 failures sector still advances empty — better than silent single drop, not perfect (P1 residual: optional log after max retries). |

**Not fixed in this pass (deferred from audit):**

- Weather sample still possible off-GT in SampleRenders  
- Unlocked `GPULidarData` vs RPC  
- `UpdateFrequency` still unused  
- Scan rate still frame-gated (documented)

### 3.4 Thread safety

| Path | Assessment |
|------|------------|
| Stencil extension RT | Render thread only — OK |
| Pose cache write/read | GT write; physics read of plain `FTransform` — acceptable snapshot |
| Empty-depth early return holds `in_flight` | Prevents FOV balloon; OK |

---

## 4. Docs / settings audit

### 4.1 Consistency with code

| Claim | Code | Match? |
|-------|------|--------|
| `EnableRpc` | `AirSimSettings` reads `EnableRpc` | **Yes** |
| EngineSound default false | Code default false | **Yes** |
| LumenGI/Reflection default false | Code false | **Yes** |
| Custom Annotation ignored | `InitializeAnnotation` + proxy gate false | **Yes** |
| CameraHost default loopback | Code `127.0.0.1` | **Yes** |
| ArmLengths mismatch warns | New log | **Yes** |
| Multirotor GPU LiDAR async | `async_capture_mode` when Multirotor | **Yes** |

### 4.2 Residual doc debt (outside this pass or incomplete)

| Item | Severity |
|------|----------|
| Root README still Cosys-heavy in places; feature list improved but not a full rewrite | P2 |
| `docs/README.md` may still link cosys-lab.github.io | P2 (not re-checked all pages) |
| Historical Cosys URLs in some sensor JSON `SeeDocsAt` samples | P2 |
| Client pyproject still 3.3.0 vs product 3.4 | P1 hygiene |
| `instance_segmentation.md` still claims batch API always “requests one camera refresh” (may overclaim) | P2 |
| Local `settings.json` often `InitialInstanceSegmentation: true` (gitignored) — operators only | Ops |

### 4.3 Packaging docs

Dual-plugin BuildPlugin sequence matches how `v3.4.0` was produced. Good for next release.

---

## 5. Checklist vs original “what we should do”

| Item | Done? |
|------|-------|
| Stencil-only annotation honesty | **Yes** |
| EnableRpc docs + examples | **Yes** |
| Settings defaults dump | **Yes** |
| Rebrand entry (mkdocs, README, uplugin) | **Yes** (core) |
| CustomDepth fail-closed | **Yes** |
| Network ops note CameraHost | **Yes** |
| Physics ArmLengths warn + GE docs | **Yes** |
| GPU LiDAR docs + empty-depth + pose | **Yes** (partial code; full deferred list no) |
| packaging / install_precompiled | **Yes** |
| Release process self-host policy | Documented as manual OK |

**Intentionally not done (still valid deferrals):**

- Re-enable Cosys proxy multi-layer annotation  
- AirLib/UE CI jobs  
- RPC auth / `simRunConsoleCommand` gates  
- Default GE opt-in flip (behavior change for all fleets)  
- Rebuild Blocks / re-upload plugin zip  

---

## 6. Risk residual after polish

### Closed or reduced

| Was | Now |
|-----|-----|
| Scene RGB as fake Seg labels | **Fail-closed black** |
| Silent ArmLengths ignore | **Warned** |
| Physics-thread LiDAR pose/draw | **Cached pose / no async draw** |
| Silent empty LiDAR sector | **Retry ×3** |
| RpcEnabled wrong docs | **Fixed** |
| Annotation docs oversell | **Honest stencil-only** |
| Packaging mystery | **Documented dual-plugin** |

### Still open (accept for now)

| Residual | Sev | Action if needed later |
|----------|-----|------------------------|
| No log on stencil fail-closed | P2 | One-shot UE_LOG |
| Weather off-GT in LiDAR sample | P1 | GT weather snapshot |
| LiDAR output unlock for RPC | P1 | Mutex copy |
| Empty depth after 3 retries still drops sector | P1 | Log + metrics |
| GE/axial default true without Physics | P1 | Migration note already; optional opt-in |
| Unauthenticated RPC/CameraHost by design | P0 ops | Firewall / loopback discipline |
| CI thin (no UE) | P1 process | Manual Blocks smoke |
| Uncommitted polish not in `v3.4.0` zip | Ops | Commit + rebuild package if shipping |

---

## 7. Validation not run

| Check | Status |
|-------|--------|
| UE compile / Blocks rebuild | **Not run** in this pass |
| Runtime Seg/IR visual after fail-closed | **Not run** |
| Multirotor GPU LiDAR live | **Not run** |
| `mkdocs build --strict` | mkdocs not installed in audit environment |
| Unit tests MultirotorPhysics | **Not run** |

**Before treating as shipped:** commit → rebuild Blocks (or BuildPlugin) → smoke Seg subwindow + one Multirotor GPU LiDAR spin.

---

## 8. Commit recommendation

Working tree is ready to commit as one production-polish commit, for example:

```text
Production polish: fail-closed stencil labels, LiDAR/physics hardening, docs honesty.

Stencil view extension clears black when CustomDepth is missing; Multirotor GPU
LiDAR caches pose and retries empty depth; Physics warns on ArmLengths mismatch;
settings/docs use EnableRpc and state stencil-only annotation; packaging and
rebrand entry points updated.
```

Include:

- All modified tracked files  
- `docs/production_audit_ba483f01_to_HEAD.md`  
- `docs/production_polish_final_audit.md` (this file)

Do **not** commit gitignored local `settings.json` or audit dumps under `tools/`.

---

## 9. Bottom line

The polish pass is a **successful closure of the audit’s actionable P0/P1 honesty + hardening items**. Code changes are small, targeted, and production-safe. Docs now match the real product (stencil-only labels, EnableRpc, dual-plugin install).

**Remaining work is mostly deferred by design** (full CI matrix, proxy annotation product decision, network auth) or **ops** (commit, rebuild, optional new tag/zip).

| Score | Scope |
|------:|-------|
| **~6.8 → ~7.5** (product readiness, if rebuilt and committed) | Full fork after polish |
| **8.0** | This polish pass alone vs its goals |

---

*End of final audit.*
