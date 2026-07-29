# Hardware Pinout — STM32F103RCT6

## I2C Buses

| Bus  | Function         | SCL  | SDA  | Devices (7-bit addr)                  |
|------|------------------|------|------|---------------------------------------|
| I2C1 | Thumb/Index | PB8 | PB9 | ADS1115 @ 0x48, 0x49 |
| I2C2 | Middle/Ring/Pinky | PB10 | PB11 | ADS1115 @ 0x48, 0x49, 0x4B |

## USB

| Function | Pin  | Note                         |
|----------|------|------------------------------|
| USB D+   | PA12 | USB CDC Virtual COM Port     |
| USB D-   | PA11 |                              |

## Debug LCD

| Function | Pin  | Note                         |
|----------|------|------------------------------|
| TBD      | TBD  | Depends on LCD model/interface |

## ADS1115 Address Configuration

Each ADS1115 address is set by connecting the ADDR pin:

| ADDR Pin → | 7-bit Address |
|------------|---------------|
| GND        | 0x48          |
| VDD        | 0x49          |
| SDA        | 0x4A          |
| SCL        | 0x4B          |

## ADS1115 Channel Assignment (all fingers identical)

| AIN Channel | Joint            | Encoder       |
|-------------|------------------|---------------|
| AIN0        | DIP              | RDC506018A    |
| AIN1        | PIP              | RDC506018A    |
| AIN2        | MCP Front (flex) | RDC506018A    |
| AIN3        | MCP Back (abd)   | RDC506018A    |

## Full Glove Finger → ADC Mapping

| Finger | ADC Bus | ADS1115 Addr | Joint Indices |
|--------|---------|--------------|---------------|
| Thumb  | I2C1    | 0x48         | 0–3           |
| Index  | I2C1    | 0x49         | 4–7           |
| Middle | I2C2    | 0x48         | 8–11          |
| Ring   | I2C2    | 0x49         | 12–15         |
| Pinky  | I2C2    | 0x4B         | 16–19         |

当前固件会在两条 I2C 总线上扫描 `0x48` 至 `0x4B`。上表的 5 个地址是完整 20 关节手套运行 `scripts/glove_calibrate.py` 与 `scripts/ads_live_monitor.py` 所要求的固定映射。
