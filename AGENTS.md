# AGENTS.md

Shared project instructions for AI coding agents.

## Build and Debug Commands

```powershell
.\make.bat                       # Default build (-Os, debug modules on)
.\make.bat clean                 # Clean build artifacts
.\make.bat flash                 # Flash via OpenOCD
.\make.bat rtt                   # RTT servers: shell 9090, waveform 9091
.\make.bat auto                  # Clean, default build, flash, start RTT
.\make.bat rttv                  # Open the RTT log viewer
mingw32-make BUILD=debug         # Unoptimized build (-O0)
mingw32-make BUILD=release       # Production build
mingw32-make TARGET_CHIP=<chip>  # Select a target explicitly
mingw32-make size                # Display ELF section sizes
```

`Makefile` and `make.bat` are authoritative. Available targets are the
subdirectories under `target/`; `Makefile` currently defaults to `at32f413`.

## AI Debugging

The authoritative procedure is `.agents/skills/aitrace/SKILL.md`.
Use `.\tools\aitrace.exe` as the single AI-facing debugging entry point.

- Start with passive RTT shell, waveform, or USB CDC capture.
- Obtain explicit confirmation before CPU halt, reset, or GDB operations.
- Build and flash only through commands defined by `Makefile` or `make.bat`.
- `tools/dev_debug.ps1` is deprecated; do not recreate or use it as the default.

## Architecture

```text
src/                  Application entry
class/                Business objects
peripheral/<chip>/    MDI hardware adaptation
peripheral/driver/    Chip-agnostic device drivers (e.g. AS5600 encoder)
foc/                  STM32G4 motor control
target/<chip>/        Target build and OpenOCD configuration
vendor/               Vendor libraries and submodules
modus/                MODUS framework submodule
tools/                AITrace, RTT viewer, and development utilities
```

Business code must access hardware through the MDI layer and must not include
vendor headers or call vendor HAL functions directly.

Use MODUS facilities instead of parallel local implementations:

- `MODUS_DECLARE_OBJECT` for object registration.
- `MODUS_SHELL_CMD` and mdebug for shell/logging support.
- `perf_counter` FSM and timing facilities for delays and cycle conversion.

## Coding Conventions

- Prefer high cohesion and low coupling.
- Keep target-specific code inside `peripheral/<chip>/` or `target/<chip>/`.
- Functions use `module_Action()` naming; types use the `_t` suffix.
- Macros use uppercase names.
- Keep embedded C/C++ source lines within 82 characters (see the embedded-coding skill). Markdown, SKILL files, design documents, reports, and other documentation use natural paragraph wrapping and are not subject to the 82-character limit.
- Preserve unrelated worktree and submodule changes.
- Do not stage, commit, switch branches, or push unless explicitly requested.

# >>> embedded-coding managed block >>>
## Embedded Coding Rules (managed by deploy-embedded-rules.ps1)

- 编码规则权威入口：`.agents/skills/embedded-coding/SKILL.md`。写/改嵌入式 C 代码前必须加载并遵守。
- MISRA 核心：显式强转；禁止符号混合比较；switch 必有 default、case 必 break；
  if-else-if 必以 else 收尾；局部变量声明即初始化；函数返回值必须检查；
  只读指针加 const；#include 在文件顶部。
- 风格：嵌入式 C/C++ 源码行宽 82 字符；文件/函数头 Doxygen（@brief/@param/@return）；注释只写"为什么"。Markdown、SKILL、设计方案、报告和其他文档不受 82 字符限制，按语义和阅读美观排版。
- 状态机：优先 perfc-PT（perfc_task_pt.h），简单状态机用裸机 switch，复杂对象用 PLOOC；
  禁止阻塞延时（用 perfc_delay_ms / perfc_is_time_out_ms）；状态切换中断保护；
  状态枚举含 IDLE/ERROR；每状态超时跳转（默认 500ms）。
- 库：对象注册用 MODUS_DECLARE_OBJECT；日志用 MODUS_SHELL_CMD/mdebug；
  日志级别约定：周期性输出用 T（可单独关），触发式事件用 I，W/E 留给异常；
  延时计时用 perf_counter；硬件访问走 MDI 层，禁止业务代码直接 include vendor HAL。
- TDD：每个模块必须有独立验证入口（tests/ 或可单独编译的测试 target）。
# <<< embedded-coding managed block <<<
