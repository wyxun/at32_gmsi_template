# FOC Framework Closeout and Sensorless Shadow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `executing-plans` to implement this plan task-by-task. This repository has
> user-owned uncommitted hardware regression work, so execution stays in the
> current workspace and does not stage or commit files.

**Goal:** Freeze a simple Motor/Core framework, make encoder-speed tuning
trustworthy, and prepare sensorless algorithms for shadow-only evaluation
without adding unmeasured work to the 20 kHz production ISR.

**Architecture:** `foc_app` remains the MODUS/product adapter; `motor_t` owns
control lifecycle, open-loop startup, feedback selection, command publication,
and control state. A single `motor_position_t` is injected into Motor. Observer
algorithms consume an explicit current/previous-voltage/timing snapshot but are
compiled and host-tested outside the production ISR until DWT timing passes.

**Tech Stack:** C11, MODUS/perf_counter, STM32G431/CORDIC, GCC host tests,
LLVM Embedded Toolchain for Arm.

---

## Scope boundaries

- Preserve the verified ADC calibration, encoder-zero alignment, stop/restart,
  reverse-speed, and PWM safety behavior from the 2026-09-10 bench report.
- Do not flash, reset, halt, or attach GDB/OpenOCD.
- Do not tune PI/filter constants without new measurements.
- Do not let SMO/NLFO select the control angle in this change.
- Do not add a multi-source registry or general runtime source manager.
- Do not stage or commit user-owned changes.

## Task 1: Restore the high-frequency build contract

**Files:**

- Modify: `Makefile`
- Modify: `foc/foc_config.h`
- Modify: `foc/app/foc_app.c`
- Test: target build command and compile-command inspection

- [ ] Replace the stale `motor_control.o` optimization rule with the actual
  `motor.o` target while retaining `foc_*.o` optimization.
- [ ] Define the STM32G431 20 kHz deadline as 8500 cycles and report maximum
  load/headroom from the existing DWT measurement outside the ISR.
- [ ] Build `debug-rel` and verify the compiler command for `motor.c` contains
  `-O2`.
- [ ] Keep the shadow observer disabled by default; hardware DWT acceptance is
  baseline max <= 5950 cycles (70%) and shadow max <= 6800 cycles (80%).

## Task 2: Make speed-mode current limiting truthful

**Files:**

- Modify: `foc/motor/motor.h`
- Modify: `foc/motor/motor.c`
- Modify: `tests/foc/test_motor.c`
- Modify: `tests/foc/test_foc_minimal_lifecycle.c`

- [ ] Add a failing Motor test in which the speed PI asks for more current than
  the command's absolute Q-current limit.
- [ ] Verify the test fails because `motor_ClockStep()` currently overwrites Q
  current using only the PID's fixed output bounds.
- [ ] Store the speed Iq limit at Start and clamp every speed-loop output to
  `[-limit, +limit]`; reject zero or out-of-range limits in speed mode.
- [ ] Add positive and negative saturation cases and run float/fixed Motor
  tests.

## Task 3: Publish Motor commands atomically

**Files:**

- Modify: `foc/motor/motor.c`
- Modify: `foc/app/foc_app.c`
- Modify: `tests/foc/test_motor.c`

- [ ] Add test instrumentation to the perf-counter port stub to prove Motor
  public mutating/query APIs enter and leave their own critical sections.
- [ ] Verify the test fails against the current caller-dependent locking.
- [ ] Protect command payload + pending marker, reference setters, lifecycle
  stop/clear, and status/feedback snapshots inside Motor.
- [ ] Remove now-redundant App-side critical sections around Motor queries.
- [ ] Run lifecycle and Motor tests in both numeric backends.

## Task 4: Close the position-provider boundary

**Files:**

- Modify: `foc/motor/motor_position.h`
- Modify: `foc/motor/motor.h`
- Modify: `foc/motor/motor.c`
- Modify: `foc/app/foc_app.h`
- Modify: `foc/app/foc_app.c`
- Modify: `foc/hal/foc_port.h`
- Modify: `peripheral/stm32g431/foc_port.c`
- Modify: `peripheral/driver/as5600.h`
- Modify: `peripheral/driver/as5600.c`
- Delete: `foc/hal/foc_sensor.h`
- Test: `tests/foc/test_motor.c`
- Test: `tests/foc/test_as5600.c`
- Test: `tests/foc/test_foc_minimal_lifecycle.c`

- [ ] Add failing tests showing voltage mode uses Motor-owned open-loop angle,
  current mode can perform Motor-owned startup alignment, and the AS5600 Motor
  provider exposes electrical plus optional mechanical feedback.
- [ ] Verify the tests fail because the current Motor provider points back to
  `foc_app_t` and the App mutates `motor_t.tCommand` during `fnRead()`.
- [ ] Move open-loop angle/ramp/alignment state and behavior into `motor_t`.
- [ ] Extend `motor_position_feedback_t` with optional mechanical feedback and
  calibration state so App diagnostics do not need a second sensor interface.
- [ ] Inject `g_tAs5600PositionOps` directly through the board position object;
  remove `foc_sensor_t`, `g_tAs5600SensorOps`, and the App-owned provider.
- [ ] Ensure provider `fnReset` is called before every normal activation and
  after stop/fault clear where stale observer state would be unsafe.
- [ ] Preserve encoder-zero capture, direction, initial offset, and background
  I2C retry cadence.
- [ ] Run AS5600, Motor, and lifecycle tests in both numeric backends.

## Task 5: Complete—but do not schedule—the observer data contract

**Files:**

- Modify: `foc/motor/motor_position.h`
- Modify: `foc/motor/motor.c`
- Modify: `tests/foc/test_motor.c`
- Modify: `foc/observer/foc_smo.h`
- Modify: `foc/observer/foc_smo.c`
- Modify: `foc/observer/foc_nlfo.h`
- Modify: `foc/observer/foc_nlfo.c`
- Create: `tests/foc/test_sensorless_observers.c`
- Modify: `tests/foc/Makefile`

- [ ] Add a failing Motor provider test that captures observer input and checks
  current alpha/beta, previous committed voltage alpha/beta, normalized bus
  voltage, and sample period.
- [ ] Populate that snapshot only when `fnObserve` exists, so the verified
  encoder fast path pays no Clarke duplication or observer cost.
- [ ] Migrate SMO/NLFO from the removed legacy position types to
  `foc_observer_input_t` and `motor_position_feedback_t`.
- [ ] Correct observer speed to angle delta divided by sample period and cover
  wraparound, reset, invalid magnitude/flux, and both numeric backends.
- [ ] Add a `sensorless-host` target that compiles and runs these algorithms,
  but do not add their sources or calls to `FOC_SOURCES`.

## Task 6: Synchronize documentation and freeze the baseline

**Files:**

- Modify: `foc/README.md`
- Modify: `foc/doc/foc-architecture.md`
- Modify: `foc/foc.mk`
- Modify: `docs/2026-09-10-foc-motor-object-refactor-bench-report.md`
- Create: `docs/2026-09-08-foc-framework-freeze-report.md`

- [ ] Replace the pre-refactor App-owned Core/Encoder diagrams with the actual
  App -> Motor -> Core/position/power-port dependency direction.
- [ ] Document that sensorless is host-ready and production-disabled until DWT
  baseline/shadow gates pass on hardware.
- [ ] Correct stale claims about `motor/` not building and `-flto` inlining.
- [ ] Record the truthful speed Iq-limit semantics and encoder tuning sequence.
- [ ] Run the full host matrix, clean STM32G431 debug-rel/release builds,
  `git diff --check`, static-state inventory, 78-column source check, and the
  encapsulation check.

## Final verification

```powershell
mingw32-make -C tests/foc clean
mingw32-make -C tests/foc minimal CC=gcc
mingw32-make -C tests/foc sensorless-host CC=gcc
mingw32-make clean
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel
mingw32-make clean
mingw32-make TARGET_CHIP=stm32g431 BUILD=release
git diff --check
tests\foc\check_motor_encapsulation.bat
```

Hardware follow-up after this implementation, requiring a separate explicit
flash/debug authorization:

1. Re-measure encoder speed-loop maximum cycles with DWT.
2. Accept baseline only at <= 5950/8500 cycles.
3. Enable one shadow observer and re-measure.
4. Accept shadow execution only at <= 6800/8500 cycles with zero deadline
   overruns; otherwise optimize by measured stage before enabling it again.
