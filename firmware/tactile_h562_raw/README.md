# tactile_h562_raw

STM32H562VGT6 的单路透明 USB-UART 调试桥工程。使用仓库根目录的 `tactile-raw-debug` 或 `tactile-raw-release` CMake preset 构建。

该目录包含构建所需的 `.ioc`、CubeMX 生成源码、驱动、中间件、链接脚本和应用代码。手写改动仅应放在 CubeMX 文件的 `/* USER CODE BEGIN */` 区块内。
