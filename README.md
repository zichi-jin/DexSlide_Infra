# DexSlide_Infra

`DexSlide_Infra` 是 DexSlide 的硬件基础设施仓库，保存 STM32 下位机固件、板级 bring-up（上电调试）资料，以及 PC 侧的 USB CDC/串口采集、校准和可视化工具。主项目 `DexSlide` 负责算法、重建、交互和应用层可视化。

仓库按“每块板子一个独立 CubeMX/CMake 工程”组织。CubeMX 生成的文件、链接脚本和驱动必须留在对应的板级目录中，不要将新工程生成到仓库根目录。

关节角采集系统的完整部署、接线、烧录、校准与故障排查见 [部署教程](docs/installation_guide.md)。

## 板卡与固件

| 工程 | MCU | 用途 | CMake preset | ELF 产物 |
| --- | --- | --- | --- | --- |
| `firmware/joints_f103/` | STM32F103RCT6 | 数据手套关节角 ADC 采集和 USB CDC 输出 | `joints-debug`、`joints-release` | `build/joints_f103/<Debug|Release>/firmware/joints_f103/dexslide_stm32.elf` |
| `firmware/tactile_h562/` | STM32H562VGT6 | 11 路电子皮肤 UART DMA 采集、USB CDC 聚合上报 | `tactile-debug`、`tactile-release` | `build/tactile_h562/<Debug|Release>/firmware/tactile_h562/tactile_h562.elf` |
| `firmware/tactile_h562_raw/` | STM32H562VGT6 | 单路 CN2/USART2 透明 USB-UART 调试桥 | `tactile-raw-debug`、`tactile-raw-release` | `build/tactile_h562_raw/<Debug|Release>/firmware/tactile_h562_raw/tactile_h562_raw.elf` |

电子皮肤复现只需按本 README 的构建与脚本入口完成最小链路验证；详细部署步骤见 [部署教程](docs/installation_guide.md)。

## 环境与工具

### Python 环境

优先复用主项目 `DexSlide` 创建的 Conda 环境：

```bash
conda activate dexslide
```

这样主项目和本仓库使用同一套 Python 依赖。若需要创建新环境或使用第三方环境，最低要求如下：

| 用途 | Python 包 |
| --- | --- |
| 所有串口/USB CDC 脚本 | `pyserial` |
| 电子皮肤解包和热力图 | `numpy`、`matplotlib` |
| 主机端测试 | `pytest` |

示例：

```bash
conda create -n dexslide python=3.11
conda activate dexslide
pip install pyserial numpy matplotlib pytest
```

### 固件工具链

- `CMake` 3.22 或更新版本。
- `Ninja`。
- ARM GNU Toolchain，确保 `arm-none-eabi-gcc` 在 `PATH` 中。
- `STM32CubeMX`：仅在修改 `.ioc` 或重新生成 STM32 工程时需要。
- `STM32CubeProgrammer` 和 ST-LINK：烧录与板级调试。
- 串口终端，例如 CuteCom、PuTTY 或 `screen`。Linux 下 CDC 设备通常为 `/dev/ttyACM*`。
- 可选：`OpenOCD`、`arm-none-eabi-gdb`，用于 SWD 单步调试。

## 构建

从仓库根目录选择一个 preset。推荐烧录前使用 `Release`：

```bash
cmake --preset tactile-release
cmake --build --preset tactile-release
```

可用 preset：

```text
joints-debug             joints-release
tactile-debug            tactile-release
tactile-raw-debug        tactile-raw-release
```

构建末尾出现 `Linking C executable ...` 且对应表格中的 `.elf` 文件生成，即表示固件构建成功。`build/`、`Debug/`、`Release/`、`.elf` 和 `.bin` 都是本地构建产物，已由 `.gitignore` 忽略，不应提交到仓库。

## 烧录与连通性检查

使用 STM32CubeProgrammer 通过 ST-LINK 的 SWD 接口选择生成的 `.elf` 后烧录。命令行示例（按本机安装路径调整）：

```bash
STM32_Programmer_CLI -c port=SWD -w build/tactile_h562/Release/firmware/tactile_h562/tactile_h562.elf -rst
```

烧录后确认以下事项：

- ST-LINK 无连接错误，复位后板卡正常运行。
- USB CDC 在 `/dev/ttyACM*` 中枚举。
- 使用目标脚本能接收数据或状态包。
- 修改了 UART、USB 或 DMA 代码时，进行对应板卡和端口的硬件回归测试。

`tactile_h562_raw` 是 CN2/USART2 的透明调试桥，专门用于单个传感器的直通验证；它不使用 11 路聚合数据格式。详见 [USART2 USB bridge debug log](docs/usart2-usb-bridge-debug-log.md)。

## 常用脚本

所有命令均在仓库根目录执行；`--port` 请替换为实际枚举的 CDC 设备。

### 关节角板

```bash
python scripts/glove_calibrate.py --port /dev/ttyACM0 --out ../DexSlide/assets/calibration/glove_calibration.json
python scripts/ads_live_monitor.py --port /dev/ttyACM0 --angles --calib-file ../DexSlide/assets/calibration/glove_calibration.json
```

`glove_calibrate.py` 将标定结果写入主项目，以便 DexSlide 应用直接读取；`ads_live_monitor.py` 用于实时检查 ADC 和关节角映射。

### 11 路电子皮肤

```bash
python scripts/tactile_stream.py --port /dev/ttyACM0
python scripts/tactile_heatmap.py --port /dev/ttyACM0 --channel adc
python scripts/tactile_heatmap.py --port /dev/ttyACM0 --channel adc --annotate
```

- `tactile_stream.py`：打印聚合流的记录数、原始字节数、帧数、速率、SEQ 丢失和固件状态。
- `tactile_heatmap.py`：同时显示 11 个 `12 x 8` 传感器阵列。`--annotate` 仅显示非零单元数值；`--annotate-all` 显示全部数值但显著增加渲染开销。
- `tactile_api.py`：可供其他 Python 程序导入的数据记录解包库，不提供直接 CLI 入口。
- `tactile_api_smoke_test.py`：主机端数据解析冒烟测试。
- `tactile_cli.py`：用于早期多 UART 控制桥和报文调试；当前 11 路采集固件的主要使用入口是 `tactile_stream.py` 与 `tactile_heatmap.py`。

使用 `python scripts/<script>.py --help` 查看脚本参数。热力图仅在安装 `numpy` 和 `matplotlib` 后可运行。

## 仓库结构

| 路径 | 内容 |
| --- | --- |
| `firmware/` | 三个相互独立的 STM32 CubeMX/CMake 工程及其板级源代码。 |
| `scripts/` | PC 侧校准、串口/USB CDC 监控、电子皮肤解包和热力图工具。 |
| `docs/` | 硬件方案、引脚/板级布局、部署教程、调试记录和 IDE 调试说明。 |
| `ChipFiles/` | 芯片、ADC 和模块参考资料。 |
| `tests/` | 不依赖硬件的主机端数据解析测试。 |
| `CMakeLists.txt`、`CMakePresets.json` | 根级工程入口和跨板级构建 preset。 |
| `.gitignore` | 构建产物、IDE 缓存和 Python 缓存的忽略规则。 |

硬件引脚、CubeMX 文件布局见 [board-layout.md](docs/board-layout.md)，目录清单见 [repository-tree.md](docs/repository-tree.md)。

## 开发约定

- 在 CubeMX 管理的文件中，仅在 `/* USER CODE BEGIN */` 与 `/* USER CODE END */` 区块中手写代码。
- 保持每块板子的 `Core/`、`Drivers/`、`Middlewares/`、`USB_*`、`.ioc`、链接脚本和 `cmake/` 文件独立，不要手工跨板复制。
- 提交前至少完成目标 preset 的 CMake 构建；涉及固件时还应记录烧录、USB 枚举和数据路径的实机验证结果。
- 主机端数据解析改动应补充或更新 `tests/` 中的 `pytest` 测试，覆盖截断数据、无效记录和计数回卷等边界情况。
