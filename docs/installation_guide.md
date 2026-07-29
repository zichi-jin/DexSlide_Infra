# 部署与复现教程

本教程以 `firmware/joints_f103/` 的关节角采集系统为主线，目标是从干净的开发机复现以下数据链路：

```text
RDC506018A 编码器 -> ADS1115 -> I2C -> STM32F103 -> USB CDC -> PC Python 脚本
```

完成后，PC 能看到 5 个 ADS1115 的 20 路原始读数；完成标定后，能输出 20 个关节角。电子皮肤部分只提供最小 bring-up 流程，不在本文公开其设备内部实现细节。

## 1. 复现目标与前置物料

### 1.1 关节角系统

需要以下硬件：

- 已装载 `STM32F103RCT6` 的 joints 控制板。
- 5 个 ADS1115 模块，或功能等价且地址、供电和 I2C 电平兼容的板卡。
- 20 路 `RDC506018A` 编码器信号，按每个 ADS1115 的 `AIN0` 至 `AIN3` 接入。
- 数据 USB 线，用于 STM32 USB CDC 与 PC 通信。
- ST-LINK/V2、ST-LINK/V3 或兼容 SWD 调试器，以及 SWD 连接线。
- 由板卡设计指定的稳定电源。

ADS1115 的 I2C 上拉电压不能高于 STM32F103 的 3.3 V GPIO 容限。若 ADS1115 模块使用 5 V 供电且把 `SCL`/`SDA` 上拉到 5 V，必须改用 3.3 V 上拉或加电平转换；所有模块必须共地。

### 1.2 主机软件

本教程以 Linux 为已验证环境。Windows 和 macOS 也可以使用相同的 CMake、ARM 工具链、CubeProgrammer 与 Python 脚本，但设备路径和安装命令不同。

优先使用主项目创建的 Conda 环境：

```bash
conda activate dexslide
```

若没有该环境，创建一个最小可用环境：

```bash
conda create -n dexslide python=3.11
conda activate dexslide
pip install pyserial numpy matplotlib pytest
```

其中关节角采集只依赖 `pyserial`；`numpy` 和 `matplotlib` 是电子皮肤热力图所需依赖。

## 2. 安装工具链

### 2.1 CMake、Ninja、ARM 编译器

在 Debian/Ubuntu 上执行：

```bash
sudo apt update
sudo apt install cmake ninja-build gcc-arm-none-eabi python3-venv
```

检查工具是否可用：

```bash
cmake --version
ninja --version
arm-none-eabi-gcc --version
python --version
```

项目要求 `CMake >= 3.22`。若发行版提供的版本过旧，请从 CMake 官方发布页安装新版，或使用 Conda：

```bash
conda install -c conda-forge cmake ninja
```

`arm-none-eabi-gcc` 必须在当前 shell 的 `PATH` 中；构建时报 `No CMAKE_C_COMPILER could be found` 时，首先检查这一项。

### 2.2 STM32CubeProgrammer 与 ST-LINK

从 STMicroelectronics 官方网站下载并安装 `STM32CubeProgrammer`。安装后确认图形界面能启动，并将其 `bin` 目录加入 `PATH`，使下面命令可运行：

```bash
STM32_Programmer_CLI --version
```

Linux 上还应确认当前用户有权限访问 ST-LINK 和 CDC 串口：

```bash
sudo usermod -aG dialout $USER
```

执行后注销并重新登录。若系统仍无法识别 ST-LINK，安装 CubeProgrammer 随附的 udev 规则后重新插拔调试器。

### 2.3 STM32CubeMX：仅在改动硬件配置时安装

正常构建和烧录不需要 CubeMX；仓库已经保存 `.ioc`、驱动、启动文件与 CMake 工程。只有要修改时钟、I2C、USB、引脚或重新生成代码时才安装 `STM32CubeMX`。

重新生成 joints 工程时，打开 `firmware/joints_f103/dexslide_stm32.ioc`，保持 `Keep User Code when re-generating` 开启。生成后检查 `Core/Src/main.c` 和 `Core/Src/stm32f1xx_hal_msp.c` 的 `/* USER CODE BEGIN */` 区块没有被覆盖，再重新构建并进行实机测试。

## 3. 获取源码并确认环境

```bash
git clone <your-DexSlide_Infra-repository-url>
cd DexSlide_Infra
conda activate dexslide
python -c "import serial; print(serial.__version__)"
cmake --list-presets
```

最后一条命令应至少列出：

```text
joints-debug
joints-release
tactile-debug
tactile-release
tactile-raw-debug
tactile-raw-release
```

`build/` 是本机构建目录，已被 Git 忽略；删除或重新创建它不会影响源码。

## 4. 关节角板接线

### 4.1 STM32 与 PC、ST-LINK

| 用途 | STM32F103 引脚 | 连接要求 |
| --- | --- | --- |
| USB D- | PA11 | 连接 USB 数据线的 D-。 |
| USB D+ | PA12 | 连接 USB 数据线的 D+。 |
| SWDIO | PA13 | 接 ST-LINK SWDIO。 |
| SWCLK | PA14 | 接 ST-LINK SWCLK。 |
| NRST | NRST | 强烈建议接 ST-LINK NRST，便于 connect-under-reset。 |
| GND | GND | 与 ST-LINK、ADS1115 和供电系统共地。 |

目标板必须已供电，且 ST-LINK 的 VTref 能检测到目标板逻辑电压。不要在不确认板卡电源设计的前提下依赖 ST-LINK 给目标板供电。

### 4.2 STM32 与 ADS1115

| 总线 | SCL | SDA | 固件 I2C 速率 |
| --- | --- | --- | --- |
| I2C1 | PB8 | PB9 | 100 kHz |
| I2C2 | PB10 | PB11 | 100 kHz |

完整五指手套的地址与脚本映射固定如下：

| 手指 | I2C 总线 | ADS1115 地址 | 编码器通道 |
| --- | --- | --- | --- |
| Thumb | I2C1 | `0x48` | `AIN0=DIP`、`AIN1=PIP`、`AIN2=MCP_front`、`AIN3=MCP_back` |
| Index | I2C1 | `0x49` | `AIN0=DIP`、`AIN1=PIP`、`AIN2=MCP_front`、`AIN3=MCP_back` |
| Middle | I2C2 | `0x48` | `AIN0=DIP`、`AIN1=PIP`、`AIN2=MCP_front`、`AIN3=MCP_back` |
| Ring | I2C2 | `0x49` | `AIN0=DIP`、`AIN1=PIP`、`AIN2=MCP_front`、`AIN3=MCP_back` |
| Pinky | I2C2 | `0x4B` | `AIN0=DIP`、`AIN1=PIP`、`AIN2=MCP_front`、`AIN3=MCP_back` |

ADS1115 地址由 `ADDR` 引脚决定：接 `GND` 为 `0x48`、接 `VDD` 为 `0x49`、接 `SDA` 为 `0x4A`、接 `SCL` 为 `0x4B`。当前固件会在两条 I2C 总线上扫描 `0x48` 至 `0x4B`，但关节角脚本只有在上述 5 个地址都出现时才会接受一整帧数据。

接线完成后，使用万用表确认每个模块的电源和公共地；断电状态下确认 `SCL` 与 `SDA` 没有短路。不要在 I2C 总线上并联多个强上拉电阻，以免上升沿过快或低电平驱动超限。

## 5. 构建关节角固件

从仓库根目录构建。初次验证可用 Debug，日常使用和烧录建议 Release：

```bash
cmake --preset joints-release
cmake --build --preset joints-release
```

成功时，末尾会出现类似 `Linking C executable dexslide_stm32.elf` 的输出，并生成：

```text
build/joints_f103/Release/firmware/joints_f103/dexslide_stm32.elf
```

常见构建失败：

| 现象 | 原因与处理 |
| --- | --- |
| `No CMAKE_C_COMPILER could be found` | 安装 ARM GNU Toolchain，并确认 `arm-none-eabi-gcc --version` 成功。 |
| CMake 版本低于 3.22 | 升级 CMake 后删除该板的本地构建目录，再重新运行 preset。 |
| `ninja: command not found` | 安装 `ninja-build`，或安装 Conda 的 `ninja`。 |
| 修改 `.ioc` 后出现未定义符号或引脚异常 | 检查是否在正确的 `firmware/joints_f103/` 内生成，并保留手写 `USER CODE`。 |

## 6. 烧录关节角固件

先把 ST-LINK 接到 SWDIO、SWCLK、GND，推荐同时接 NRST；再给目标板供电。可在 CubeProgrammer 图形界面中选择 `ST-LINK`、连接、选择上一节生成的 `.elf`，再点击 Download。

如已将 CLI 加入 `PATH`，可用：

```bash
STM32_Programmer_CLI -c port=SWD -w build/joints_f103/Release/firmware/joints_f103/dexslide_stm32.elf -v -rst
```

成功标准：

- CubeProgrammer/CLI 能识别目标 MCU 和目标电压。
- 写入结束没有 `Error`，并且 `-v` 校验通过。
- `-rst` 后程序开始运行，USB 重新枚举为 CDC 串口。

烧录或连接失败的排查顺序：

1. 确认目标板已供电，ST-LINK 与目标板 GND 共地，且 VTref 有正确电压。
2. 复核 SWDIO、SWCLK 没有接反，排除外设对 PA13/PA14 的强驱动。
3. 接上 NRST，并在 CubeProgrammer 中尝试 `Connect under reset`；CLI 可尝试 `-c port=SWD mode=UR reset=HWrst`。
4. 将 SWD 频率降低到 100 kHz 或更低，缩短调试线。
5. 若仍无法附着，断开非必要外设和 USB，仅保留供电与 SWD 后再试。

不要依赖其他开发机的本地路径包装脚本。新机器应优先用官方 CubeProgrammer GUI 或已验证的 `STM32_Programmer_CLI`。

## 7. 验证 USB CDC 与原始采集

烧录后使用数据 USB 线连接 PC。Linux 下可观察内核日志：

```bash
dmesg --follow
```

另开终端确认设备：

```bash
ls -l /dev/ttyACM*
```

假设枚举为 `/dev/ttyACM0`，运行原始监视器：

```bash
conda activate dexslide
python scripts/ads_live_monitor.py --port /dev/ttyACM0
```

健康输出由一行或多行 ADS1115 记录组成，形式如下：

```text
I2C1@0x48[A0:123,A1:456,A2:789,A3:101] | I2C1@0x49[...] | I2C2@0x48[...] | I2C2@0x49[...] | I2C2@0x4B[...]
```

转动某一个关节时，对应的 `A0` 至 `A3` 原始值应持续变化。固件每秒重新发现一次 ADS1115；因此刚上电、重新插接模块或恢复 I2C 后，最多等待约 1 秒再判断设备不存在。

| 现象 | 优先检查 |
| --- | --- |
| 没有 `/dev/ttyACM*` | USB 数据线是否仅供电、PA11/PA12 走线、固件是否成功运行、`dmesg --follow` 是否报 USB 枚举错误。 |
| `Permission denied: /dev/ttyACM0` | 执行 `sudo usermod -aG dialout $USER` 并重新登录。 |
| 串口被占用 | 关闭 CuteCom、`screen`、其他 Python 脚本或 IDE 的串口监视器。一个 CDC 端口一次只能由一个程序打开。 |
| 输出 `no active ADS1115 in cached set` | 依次检查 ADS1115 供电、共地、I2C 引脚、地址配置和上拉电压；固件只扫描 `0x48` 至 `0x4B`。 |
| 只出现部分 ADS1115 | 检查该模块的 `ADDR` 配置和所在总线；完整校准必须有固定的 5 个地址。 |
| 读数固定不变或变化异常 | 检查对应编码器的 AIN 接线、模拟地、供电和关节机械连接。 |

如需只查看文本流，也可使用 `screen /dev/ttyACM0 115200`。退出 `screen` 后再启动 Python 脚本，避免端口互斥。

## 8. 标定并输出关节角

原始 ADC 值不等于角度。每个关节需至少采集 `0 deg` 与参考角度，以建立线性映射。推荐把校准结果写到主项目，方便 DexSlide 应用直接加载：

```bash
python scripts/glove_calibrate.py \
  --port /dev/ttyACM0 \
  --out ../DexSlide/assets/calibration/glove_calibration.json
```

脚本依次处理 20 个关节。对每一项：

1. 把关节放到机械零位，按 Enter 捕获 `0deg`。
2. 把关节放到 90 度，按 Enter 捕获参考值。
3. 对非拇指的 `MCP_back`，可输入实际参考角度、直接 Enter 使用 90 度，或输入 `skip` 使用默认比例。
4. 每完成一项会立即写入 JSON；中断后再次使用同一 `--out` 路径可继续未完成项。

标定完成后验证角度输出：

```bash
python scripts/ads_live_monitor.py \
  --port /dev/ttyACM0 \
  --angles \
  --calib-file ../DexSlide/assets/calibration/glove_calibration.json
```

健康输出应包含 20 个 `<finger>.<joint>:<angle>` 项，转动关节时只有对应项显著变化。若标定脚本长期显示 `no full frame available yet`，说明它没有收到固定 5 个 ADS1115 地址组成的完整帧，应先回到上一节修复接线和地址，而不是重复标定。

## 9. SWD 断点调试

当固件已写入但 USB CDC 不枚举或 I2C 行为异常时，使用 SWD 比依赖串口日志更可靠：

1. 保持 SWDIO、SWCLK、GND、NRST 连接。
2. 使用 CubeProgrammer 确认能连接和复位目标。
3. 在 VS Code 安装 Cortex-Debug，并确认 `arm-none-eabi-gdb`、`ST-LINK_gdbserver` 可用。
4. 在 `main`、`MX_USB_DEVICE_Init`、`Error_Handler` 设置断点，确认初始化是否到达主循环。
5. 检查 `hi2c1`、`hi2c2` 的 HAL error/state，以及 `g_i2c1_active_mask`、`g_i2c2_active_mask` 是否包含预期地址位。

已有的 H562 调试环境说明在 [vscode-debug.md](vscode-debug.md)。它包含本机路径示例，不应直接复制到其他开发机；关节角板同样适用 SWD 接线和“先确认物理链路、再看软件”的排查原则。

## 10. 电子皮肤最小复现

电子皮肤硬件和传感器由供应方提供。本仓库公开部分只覆盖板级固件的构建、烧录和 PC 侧可见性验证。

1. 按板上连接器丝印接好传感器、H562 板的 USB 数据线，以及 ST-LINK 的 SWDIO、SWCLK、GND、NRST；所有设备共地。
2. 构建并烧录聚合采集固件：

```bash
cmake --preset tactile-release
cmake --build --preset tactile-release
STM32_Programmer_CLI -c port=SWD -w build/tactile_h562/Release/firmware/tactile_h562/tactile_h562.elf -v -rst
```

3. 确认 H562 的 USB CDC 设备号后，检查持续数据和状态：

```bash
python scripts/tactile_stream.py --port /dev/ttyACM0
```

4. 显示最终实时效果：

```bash
python scripts/tactile_heatmap.py --port /dev/ttyACM0 --channel adc
```

如需显示被激活单元的数字，增加 `--annotate`。若没有 CDC 设备、没有记录或热图不变化，先依次确认 USB 枚举、供电/共地、连接器方向、烧录的是否为 `tactile_h562.elf`，再联系设备提供方确认传感器端配置。
