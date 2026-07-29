# tactile_h562

STM32H562VGT6 的多路触觉采集固件工程。使用仓库根目录的 `tactile-debug` 或 `tactile-release` CMake preset 构建；不要在此目录外生成 CubeMX 文件。

该目录包含构建所需的 `.ioc`、CubeMX 生成源码、驱动、中间件、链接脚本和应用代码。手写改动仅应放在 CubeMX 文件的 `/* USER CODE BEGIN */` 区块内。
