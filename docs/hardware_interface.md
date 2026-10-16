# ESP32 硬件接口说明

## 1. 引脚汇总

| 接口 | 信号 | ESP32 GPIO | 方向（相对 ESP32） | 说明 |
|---|---|---:|---|---|
| W5500 | RSTN | GPIO8 | 输出 | W5500 复位，低有效 |
| W5500 | INTN | GPIO9 | 输入 | W5500 中断，低有效 |
| W5500 | MOSI | GPIO10 | 输出 | SPI 主机输出 |
| W5500 | MISO | GPIO11 | 输入 | SPI 主机输入 |
| W5500 | SCLK | GPIO12 | 输出 | SPI 时钟 |
| W5500 | SCSN | GPIO13 | 输出 | SPI 片选，低有效 |
| 双色 LED | LED 通道 1 | GPIO42 | 输出 | 低电平点亮 |
| 双色 LED | LED 通道 2 | GPIO35 | 输出 | 低电平点亮 |
| CAN | CAN1_TX | GPIO15 | 输出 | 接 CAN 收发芯片 TXD |
| CAN | CAN1_RX | GPIO16 | 输入 | 接 CAN 收发芯片 RXD |
| RS485 | UART1_TX | GPIO17 | 输出 | 接 RS485 收发芯片 D |
| RS485 | UART1_RX | GPIO18 | 输入 | 接 RS485 收发芯片 R |
| RS485 | U1_RE# | GPIO21 | 输出 | RS485 收发方向控制 |
| UART0 | UART0_TX | GPIO43 | 输出 | 普通串口发送 |
| UART0 | UART0_RX | GPIO44 | 输入 | 普通串口接收 |
| UART2 | UART2_TX | GPIO33 | 输出 | 普通串口发送 |
| UART2 | UART2_RX | GPIO34 | 输入 | 普通串口接收 |

## 2. W5500 网口

以太网控制器使用 **W5500**，与 ESP32 通过 SPI 连接。

| W5500 信号 | ESP32 GPIO | 说明 |
|---|---:|---|
| RSTN | GPIO8 | 硬件复位，低有效 |
| INTN | GPIO9 | 中断输入，低有效 |
| MOSI | GPIO10 | SPI MOSI |
| MISO | GPIO11 | SPI MISO |
| SCLK | GPIO12 | SPI CLK |
| SCSN | GPIO13 | SPI CS，低有效 |

程序侧只需要按照以上 GPIO 配置 SPI、复位和中断信号。

## 3. 双色 LED

板上双色 LED 使用两个 GPIO 独立控制。

| 信号 | ESP32 GPIO | 有效电平 |
|---|---:|---|
| LED 通道 1 | GPIO42 | 低有效 |
| LED 通道 2 | GPIO35 | 低有效 |

LED 公共端连接 3.3 V，因此 GPIO 输出低电平时对应通道点亮，输出高电平时熄灭。

原理图未明确标注 GPIO42、GPIO35 分别对应哪一种颜色，程序中建议按“通道 1 / 通道 2”定义，实际颜色可通过硬件确认。

## 4. CAN 接口

CAN 收发芯片为 **ISO1044BDR**。

| 信号 | ESP32 GPIO | 收发芯片引脚 | 方向 |
|---|---:|---|---|
| CAN1_TX | GPIO15 | TXD | ESP32 → ISO1044BDR |
| CAN1_RX | GPIO16 | RXD | ISO1044BDR → ESP32 |

总线侧信号为：

- `CAN1_H`
- `CAN1_L`

CAN 接口不需要额外的收发方向控制 GPIO。

## 5. RS485 接口

RS485 收发芯片为 **THVD1410DR**，采用半双工收发方式。

| 信号 | ESP32 GPIO | THVD1410DR 引脚 | 方向 |
|---|---:|---|---|
| UART1_TX | GPIO17 | D | ESP32 → THVD1410DR |
| UART1_RX | GPIO18 | R | THVD1410DR → ESP32 |
| U1_RE# | GPIO21 | RE# / DE | ESP32 → THVD1410DR |

总线侧信号为：

- `485_1_A`
- `485_1_B`

原理图中 `RE#` 和 `DE` 使用同一个方向控制信号：

| GPIO21 | 工作状态 |
|---|---|
| 低电平 | 接收 |
| 高电平 | 发送 |

因此程序设计时，需要将 GPIO21 作为 RS485 收发方向控制引脚。

## 6. 普通串口

### UART0

| 信号 | ESP32 GPIO | 方向 |
|---|---:|---|
| UART0_TX | GPIO43 | 输出 |
| UART0_RX | GPIO44 | 输入 |

### UART2

| 信号 | ESP32 GPIO | 方向 |
|---|---:|---|
| UART2_TX | GPIO33 | 输出 |
| UART2_RX | GPIO34 | 输入 |

普通串口部分只需按照以上 TX/RX GPIO 进行串口配置。
