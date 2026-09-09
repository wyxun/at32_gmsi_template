# FOC Framework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the single-motor FOC framework around one Motor owner while
preserving the verified ADC/PWM mappings, numeric backends, and encoder path.

**Architecture:** `foc_app` remains MODUS/product glue and owns one embedded
`motor_t`. Motor owns lifecycle, references, control state, position provider,
current-cycle input, and the 20 kHz divider. FocCore stays deterministic and
hardware-independent; ADC, PWM, and position providers remain injected ops.

**Tech Stack:** C11/gnu11 host tests, existing MODUS/perf_counter runtime,
float and fixed FOC scalar backends, STM32G431 MDI adapters.

---

### Task 1: Freeze and test the framework contracts

**Files:**
- Modify: `foc/foc_types.h`
- Modify: `foc/hal/foc_port.h`
- Modify: `foc/motor/motor_position.h`
- Test: `tests/foc/test_motor.c`
- Test: `tests/foc/test_foc_minimal_lifecycle.c`

- [ ] **Step 1: Add contract assertions for the four control modes and the
  separate ADC/PWM operations.** Keep `foc_core_command_t` internal-facing,
  but add `FOC_MODE_POSITION` and make validation accept exactly the four
  modes. Do not change the ADC sample expressions or PWM operation members.

- [ ] **Step 2: Replace the position provider table with the current contract:
  `fnInit(context, motor parameters)`, `fnReset`, `fnPoll`,
  `fnReadFeedback`, and optional `fnCaptureZero`. Remove observer input and
  `fnObserve` from this current interface.

- [ ] **Step 3: Extend host stubs to record polling, cached feedback reads,
  zero capture, and per-instance state. Add tests that prove a provider poll
  is available only from the foreground path and that two Motor objects do
  not share runtime state.

- [ ] **Step 4: Run the focused host tests in both numeric modes and confirm
  the existing core/encoder tests remain green before changing implementation.

  Run: `mingw32-make -C tests/foc minimal-core minimal-encoder motor-float motor-fixed`

  Expected: all four executables exit 0 and report `PASS`.

### Task 2: Make Motor the sole domain owner

**Files:**
- Modify: `foc/motor/motor.h`
- Modify: `foc/motor/motor.c`
- Modify: `foc/middleware/foc_core.h`
- Modify: `foc/middleware/foc_core.c`
- Test: `tests/foc/test_motor.c`

- [ ] **Step 1: Replace `MOTOR_STATE_CALIBRATING` with explicit
  `MOTOR_STATE_ADC_CAL` and retain `MOTOR_STATE_POSITION_CAL`; include
  `IDLE` and `FAULT` in the lifecycle enum. Add `FOC_MODE_POSITION` to core
  command validation.

- [ ] **Step 2: Add Motor-owned typed reference fields and APIs:
  `motor_Start(motor, mode)`, `motor_SetVoltageReference`,
  `motor_SetCurrentReference`, `motor_SetSpeedReference`,
  `motor_SetPositionReference`, `motor_RequestAdcCalibration`,
  `motor_RequestPositionCalibration`, `motor_ClearFault`, and coherent
  `motor_GetStatus`. Remove the command mailbox and all distributed command
  retry/BUSY machinery.

- [ ] **Step 3: Implement one short interrupt critical section per setter and
  status copy. The section may only copy validated scalar fields; it must not
  call algorithms, hardware, wait, or return BUSY. While IDLE, setters preload
  their own fields; while RUNNING, only the active mode's setter succeeds.

- [ ] **Step 4: Move speed PI execution into `motor_HighFrequencyStep` behind
  a modulo-20 divider. The high-frequency owner becomes the only writer of
  speed PI state, `Iq_ref`, current PI inputs, and PWM commands. Preserve the
  configured PI parameters and clamp speed PI output to the Motor Iq limit.

- [ ] **Step 5: Move the phase current input into persistent
  `motor_t.tCycleInput`, update it in place each cycle, and remove
  `qIuLatest/qIvLatest/qIwLatest`. Keep alignment at the fixed electrical zero
  with the configured bounded D-axis current and completion timeout.

- [ ] **Step 6: Add focused tests for ADC_CAL never enabling PWM, ALIGN
  entering only after ADC calibration, stop/fault/timeout exits, typed setter
  mode rules, divider behavior, and two-instance PI independence. Run
  `mingw32-make -C tests/foc motor-float motor-fixed`.

### Task 3: Reduce the App to glue and scheduling

**Files:**
- Modify: `foc/app/foc_app.h`
- Modify: `foc/app/foc_app.c`
- Modify: `tests/foc/test_foc_minimal_lifecycle.c`
- Modify: `foc/foc.mk`

- [ ] **Step 1: Change `foc_app_cfg_t` to contain Motor configuration and the
  MODUS ring-buffer/diagnostic configuration only. Remove App-owned sensor,
  position, startup, control, fault, and status structures.

- [ ] **Step 2: Bind the configured position provider directly into Motor and
  make `foc_app_Init` perform only object initialization, MODUS registration,
  and diagnostics setup. Keep the generated single App object, but ensure the
  App does not expose mirrored Start/Stop/reference/status APIs.

- [ ] **Step 3: Route shell handlers directly to Motor APIs. Keep product
  parsing and logging in App, including the existing encoder calibration
  command, but remove `foc_app_Start`, `foc_app_Stop`, reference wrappers,
  `foc_app_GetStatus`, and the duplicate `foc_status_t`.

- [ ] **Step 4: Make `foc_app_Clock` timekeeping-only. Make `foc_app_Run` the
  only caller of `motor_BackgroundStep`; it owns foreground position polling
  and timestamp-based 1 kHz rate limiting. The 1 ms Clock and 20 kHz ISR must
  never call a blocking sensor transaction.

- [ ] **Step 5: Update tests to query `motor_GetStatus` and `motor_GetFeedback`
  directly through the embedded Motor. Verify no duplicate App domain state or
  public mirrored control API remains with `rg` checks.

### Task 4: Adapt the encoder provider without changing hardware behavior

**Files:**
- Modify: `peripheral/driver/as5600.h`
- Modify: `peripheral/driver/as5600.c`
- Modify: `peripheral/stm32g431/foc_port.c`
- Modify: `foc/hal/foc_port.h`
- Test: `tests/foc/test_as5600.c`

- [ ] **Step 1: Expose AS5600 as a Motor position provider with `fnPoll` for
  blocking I2C refresh and `fnReadFeedback` for cached, non-blocking reads.
  Keep the encoder extrapolation/filter runtime inside each AS5600 instance.

- [ ] **Step 2: Preserve electrical-zero capture, pole-pair conversion,
  direction inversion, invalid-sample behavior, and the existing 1 kHz failure
  backoff. Do not add observer inputs to the physical position interface.

- [ ] **Step 3: Keep `foc_port.c` ADC/PWM mapping expressions unchanged:
  ADC1 injected 0 → U, ADC2 injected 1 → V, ADC2 injected 0 → W, polarity
  `offset - raw`, TIM1 CH4 trigger, and U/V/W → TIM1 CH1/2/3.

- [ ] **Step 4: Run float/fixed AS5600 tests and the target compile so the
  adapter contract and frozen board mapping are checked together.

### Task 5: Bind diagnostics to production state and complete verification

**Files:**
- Modify: `foc/app/foc_app.c`
- Modify: `foc/app/foc_app.h`
- Modify: `tests/foc/test_motor.c`
- Modify: `tests/foc/test_foc_minimal_lifecycle.c`
- Modify: `foc/doc/foc-architecture.md`

- [ ] **Step 1: Bind mwaveform phase-current channels directly to
  `&motor.tCycleInput.qIu`, `.qIv`, and `.qIw`. Register only object-lifetime
  addresses; do not register stack locals or waveform-only mirrors.

- [ ] **Step 2: Add a test-visible check that each waveform address is inside
  the owning Motor object and that `mwaveform.Step` observes completed cycle
  input values. Keep existing FocCore/Motor channels unchanged where they are
  real production state.

- [ ] **Step 3: Update the architecture document to describe the final
  contracts and scheduling ownership, without adding future observer/mode
  implementation.

- [ ] **Step 4: Run the complete host matrix:
  `mingw32-make -C tests/foc all`, then build the default target with
  `.\make.bat` and both `mingw32-make BUILD=debug` and
  `mingw32-make BUILD=release` where the toolchain is available.

- [ ] **Step 5: Run static contract checks for forbidden App wrappers, old
  position-op names, mutable file-static algorithm state, unchanged ADC/PWM
  mapping expressions, and direct Clock/background violations. Record any
  unavailable hardware DWT measurement as an explicit verification gap rather
  than claiming the timing gate.
