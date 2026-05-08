# Hardware Pinout — STM32F103RCT6

## I2C Buses

| Bus  | Function         | SCL  | SDA  | Devices (7-bit addr)                  |
|------|------------------|------|------|---------------------------------------|
| I2C1 | Thumb/Index/Mid  | PB6  | PB7  | ADS1115 @ 0x48, 0x49, 0x4A           |
| I2C2 | Ring/Pinky       | PB10 | PB11 | ADS1115 @ 0x48, 0x49                  |

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

## Finger → ADC Mapping

| Finger | ADC Bus | ADS1115 Addr | Joint Indices |
|--------|---------|--------------|---------------|
| Thumb  | I2C1    | 0x48         | 0–3           |
| Index  | I2C1    | 0x49         | 4–7           |
| Middle | I2C1    | 0x4A         | 8–11          |
| Ring   | I2C2    | 0x48         | 12–15         |
| Pinky  | I2C2    | 0x49         | 16–19         |
