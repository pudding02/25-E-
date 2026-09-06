---
name: "lushan-pi-k230-pinout"
description: "立创庐山派K230-CanMV开发板完整引脚图速查(40Pin排针/GH1.25座子/GPIO0-63复用表)。当需要查询K230引脚定义、物理Pin与GPIO映射、UART/I2C/PWM引脚分配、供电引脚时调用。"
---

# 立创·庐山派 K230-CanMV 引脚图速查

> 适用板卡：立创·庐山派 K230-CanMV 开发板（1GB 标准版）。Lite 版（128MB）有 7 处引脚差异，见文末说明。

---

## 一、40Pin 排针引脚图（兼容树莓派物理布局）

庐山派 40Pin 排针**兼容树莓派物理布局**，但 GPIO 编号与树莓派不同。所有 IO 电平 **3.3V**，不可接 5V 信号。

```
┌─────────────┬──────────┬──────┬──────────┬─────────────┐
│  左列(奇)   │  GPIO    │ Pin  │ Pin      │  GPIO/功能   │
├─────────────┼──────────┼──────┼──────────┼─────────────┤
│ 3.3V        │ ─        │  1   │  2       │ 5V          │
│ I2C1_SDA    │ GPIO41   │  3   │  4       │ 5V          │
│ I2C1_SCL    │ GPIO40   │  5   │  6       │ GND         │
│ JTAG_TCK    │ GPIO2    │  7   │  8       │ GPIO3 UART1_TX│
│ GND         │ ─        │  9   │ 10       │ GPIO4 UART1_RX│
│ UART2_TXD   │ GPIO5    │ 11   │ 12       │ GPIO23      │
│ UART2_RXD   │ GPIO6    │ 13   │ 14       │ GND         │
│ OSPI_D6     │ GPIO22   │ 15   │ 16       │ GPIO18 OSPI_D2│
│ 3.3V        │ ─        │ 17   │ 18       │ GPIO24 OSPI_DQS│
│ QSPI0_D0    │ GPIO16   │ 19   │ 20       │ GND         │
│ QSPI0_CS0   │ GPIO14   │ 21   │ 22       │ GPIO25 PWM5  │
│ QSPI0_CLK   │ GPIO15   │ 23   │ 24       │ GPIO17 OSPI_D1│
│ GND         │ ─        │ 25   │ 26       │ GPIO19 OSPI_D3│
│ I2C0_SDA    │ GPIO49   │ 27   │ 28       │ GPIO48 I2C0_SCL│
│ PWM2/I2C4   │ GPIO7    │ 29   │ 30       │ GND         │
│ PWM3/I2C4   │ GPIO8    │ 31   │ 32       │ GPIO20 OSPI_D4│
│ PWM4/UART1  │ GPIO9    │ 33   │ 34       │ GND         │
│ UART1_RX    │ GPIO10   │ 35   │ 36       │ GPIO21 OSPI_D5│
│ GPIO        │ GPIO11   │ 37   │ 38       │ GPIO12 M_CLK1│
│ GND         │ ─        │ 39   │ 40       │ GPIO13 M_CLK1│
└─────────────┴──────────┴──────┴──────────┴─────────────┘
```

**供电脚汇总**：
- 3.3V：Pin 1、Pin 17
- 5V：Pin 2、Pin 4
- GND：Pin 6、9、14、20、25、30、34、39

> ⚠️ 排针上**没有 ADC**。ADC 仅支持最高 1.8V，通过 FPC 排线座单独引出（4 通道）。

---

## 二、GH1.25-4P 串口座子（板上 3 个，带锁）

线序统一（从左到右 1→4）：

| 脚位 | 信号 | 说明 |
|------|------|------|
| 1 | **5V** | 电源正极 |
| 2 | RXD | 接外部 TXD |
| 3 | TXD | 接外部 RXD |
| 4 | **GND** | 电源地 |

三个座子对应的串口：

| 座子 | 串口 | GPIO | CanMV 中可用 |
|------|------|------|:---:|
| 串口0 | UART0 | TXD=GPIO38, RXD=GPIO39 | ❌ 系统占用（RT-Smart 调试） |
| 串口2 | UART2 | TXD=GPIO11, RXD=GPIO12 | ✅ 可用（也可复用 I2C2） |
| 串口3 | UART3 | TXD=GPIO50, RXD=GPIO51 | ✅ 可用 |

> 座子和背面 40Pin 排针焊盘是**同一组信号**，不能同时接不同设备。
> 串口通信交叉接线：开发板 RXD 接对方 TXD，开发板 TXD 接对方 RXD。

---

## 三、GPIO 0-63 复用功能表

| pin | 默认功能 | 可复用功能 |
|-----|----------|-----------|
| 0 | GPIO0 | BOOT0 |
| 1 | GPIO1 | BOOT1 |
| 2 | JTAG_TCK | GPIO2 / PULSE_CNTR0 |
| 3 | JTAG_TDI | GPIO3 / PULSE_CNTR1 / UART1_TXD |
| 4 | JTAG_TDO | GPIO4 / PULSE_CNTR2 / UART1_RXD |
| 5 | UART2_TXD | GPIO5 / JTAG_TMS / PULSE_CNTR3 |
| 6 | UART2_RXD | GPIO6 / JTAG_RST / PULSE_CNTR4 |
| 7 | PWM2 | GPIO7 / IIC4_SCL |
| 8 | PWM3 | GPIO8 / IIC4_SDA |
| 9 | PWM4 | GPIO9 / UART1_TXD / IIC1_SCL |
| 10 | CTRL_IN_3D | GPIO10 / UART1_RXD / IIC1_SDA |
| 11 | CTRL_O1_3D | GPIO11 / UART2_TXD / IIC2_SCL |
| 12 | CTRL_O2_3D | GPIO12 / UART2_RXD / IIC2_SDA |
| 13 | M_CLK1 | GPIO13 |
| 14 | OSPI_CS | GPIO14 / QSPI0_CS0 |
| 15 | OSPI_CLK | GPIO15 / QSPI0_CLK |
| 16 | OSPI_D0 | GPIO16 / QSPI1_CS4 / QSPI0_D0 |
| 17 | OSPI_D1 | GPIO17 / QSPI1_CS3 / QSPI0_D1 |
| 18 | OSPI_D2 | GPIO18 / QSPI1_CS2 / QSPI0_D2 |
| 19 | OSPI_D3 | GPIO19 / QSPI1_CS1 / QSPI0_D3 |
| 20 | GPIO20 | OSPI_D4 / QSPI1_CS0 / PULSE_CNTR0 |
| 21 | OSPI_D5 | GPIO21 / QSPI1_CLK / PULSE_CNTR1 |
| 22 | OSPI_D6 | GPIO22 / QSPI1_D0 / PULSE_CNTR2 |
| 23 | GPIO23 | OSPI_D7 / QSPI1_D1 / PULSE_CNTR3 |
| 24 | GPIO24 | OSPI_DQS / QSPI1_D2 / PULSE_CNTR4 |
| 25 | GPIO25 | PWM5 / QSPI1_D3 / PULSE_CNTR5 |
| 26 | PDM_CLK | GPIO26 / MMC1_CLK |
| 27 | GPIO27 | MMC1_CMD / PULSE_CNTR5 / PDM_IN0 |
| 28 | GPIO28 | MMC1_D0 / UART3_TXD / PDM_IN1 |
| 29 | GPIO29 | MMC1_D1 / UART3_RXD / CTRL_IN_3D |
| 30 | GPIO30 | MMC1_D2 / UART3_RTS / CTRL_O1_3D |
| 31 | GPIO31 | MMC1_D3 / UART3_CTS / CTRL_O2_3D |
| 32 | IIS_CLK | GPIO32 / IIC0_SCL / UART3_TXD |
| 33 | IIS_WS | GPIO33 / IIC0_SDA / UART3_RXD |
| 34 | IIS_D_IN0 | GPIO34 / IIC1_SCL / PDM_IN3 / UART3_RTS |
| 35 | IIS_D_OUT0 | GPIO35 / IIC1_SDA / PDM_IN1 / UART3_CTS |
| 36 | GPIO36 | IIC3_SCL / IIS_D_IN1 / PDM_IN2 / UART4_TXD |
| 37 | GPIO37 | IIC3_SDA / IIS_D_OUT1 / PDM_IN0 / UART4_RXD |
| 38 | UART0_TXD | GPIO38 / QSPI1_CS0 / HSYNC0 |
| 39 | UART0_RXD | GPIO39 / QSPI1_CLK / VSYNC0 |
| 40 | IIC1_SCL | GPIO40 / UART1_TXD / QSPI1_D0 |
| 41 | IIC1_SDA | GPIO41 / UART1_RXD / QSPI1_D1 |
| 42 | GPIO42 | UART1_RTS / PWM0 / QSPI1_D2 |
| 43 | GPIO43 | UART1_CTS / PWM1 / QSPI1_D3 |
| 44 | IIC3_SCL | GPIO44 / UART2_TXD / SPI2AXI_CK |
| 45 | IIC3_SDA | GPIO45 / UART2_RXD / SPI2AXI_CS |
| 46 | IIC4_SCL | GPIO46 / UART2_RTS / PWM2 |
| 47 | IIC4_SDA | GPIO47 / UART2_CTS / PWM3 |
| 48 | IIC0_SCL | GPIO48 / UART4_TXD / SPI2AXI_DI |
| 49 | IIC0_SDA | GPIO49 / UART4_RXD / SPI2AXI_DO |
| 50 | UART3_TXD | GPIO50 / IIC2_SCL / QSPI0_CS4 |
| 51 | UART3_RXD | GPIO51 / IIC2_SDA / QSPI0_CS3 |
| 52 | GPIO52 | UART3_RTS / PWM4 / IIC3_SCL |
| 53 | GPIO53 | UART3_CTS / PWM5 / IIC3_SDA |
| 54 | MMC1_CMD | GPIO54 / QSPI0_CS0 / PWM0 |
| 55 | MMC1_CLK | GPIO55 / QSPI0_CLK / PWM1 |
| 56 | MMC1_D0 | GPIO56 / QSPI0_D0 / PWM2 |
| 57 | MMC1_D1 | GPIO57 / QSPI0_D1 / PWM3 |
| 58 | MMC1_D2 | GPIO58 / QSPI0_D2 / PWM4 |
| 59 | MMC1_D3 | GPIO59 / QSPI0_D3 / PWM5 |
| 60 | GPIO60 | PWM0 / IIC0_SCL / QSPI0_CS2 / HSYNC1 |
| 61 | GPIO61 | PWM1 / IIC0_SDA / QSPI0_CS1 / VSYNC1 |
| 62 | M_CLK2 | GPIO62 / UART3_DE |
| 63 | M_CLK3 | GPIO63 / UART3_RE |

---

## 四、常用外设引脚速查

### UART（串口）

| 串口 | TXD GPIO | RXD GPIO | CanMV 可用 | 物理位置 |
|------|----------|----------|:---:|------|
| UART0 | 38 | 39 | ❌ | GH1.25 串口0座 |
| UART1 | 3 / 9 | 4 / 10 | ✅ | 40Pin Pin8/Pin10 或 Pin9? 见复用表 |
| UART2 | 5 / 11 / 44 | 6 / 12 / 45 | ✅ | 40Pin Pin11/Pin13 + GH1.25 串口2座 |
| UART3 | 28 / 32 / 50 | 29 / 33 / 51 | ✅ | GH1.25 串口3座 |
| UART4 | 36 / 48 | 37 / 49 | ✅ | 需用 FPIOA 映射 |

> UART2 默认：TXD=GPIO5(Pin11)、RXD=GPIO6(Pin13)。代码中 `FPIOA.UART2_TXD` 映射到 GPIO5。

### I2C

| 总线 | SCL GPIO | SDA GPIO | 说明 |
|------|----------|----------|------|
| I2C0 | 32 / 48 / 60 | 33 / 49 / 61 | 40Pin Pin27/Pin28，板载已有上拉 |
| I2C1 | 9 / 34 / 40 | 10 / 35 / 41 | 40Pin Pin3/Pin5，板载已有上拉 |
| I2C2 | 11 / 50 | 12 / 51 | 与 UART2 复用 |
| I2C3 | 36 / 44 / 52 | 37 / 45 / 53 | |
| I2C4 | 7 / 46 | 8 / 47 | |

### PWM（6 路）

| PWM | 可选 GPIO |
|-----|-----------|
| PWM0 | 42 / 54 / 60 |
| PWM1 | 43 / 55 / 61 |
| PWM2 | 7 / 46 / 56 |
| PWM3 | 8 / 47 / 57 |
| PWM4 | 9 / 52 / 58 |
| PWM5 | 25 / 53 / 59 |

### 板载资源占用

| 资源 | GPIO | 说明 |
|------|------|------|
| RGB 红灯 | 62 | |
| RGB 蓝灯 | 63 | |
| 用户按键 | 21 | |
| 蜂鸣器 | — | 4000Hz |

---

## 五、其他接口

| 接口 | 说明 |
|------|------|
| **ADC** | 4 路，FPC 排线座引出，最高 1.8V 输入 |
| **8-24V 电源** | GH1.25-2P 自锁座，可接 3S 锂电池，DCDC 转 5V |
| **Type-C** | USB0，5V 供电 + CanMV 数据 |
| **USB HOST** | USB1，Type-A，可接 U 盘/以太网 |
| **MIPI CSI0/1/2** | 三路摄像头，兼容树莓派摄像头接口 |
| **MIPI DSI** | 屏幕接口 |
| **TF 卡** | 自弹式 MicroSD 卡座 |
| **WIFI** | 2.4G，板载陶瓷天线，可切 IPEX |

---

## 六、供电方式（三路输入，自动切换）

1. **Type-C（USB 5V）**：电脑或适配器供电
2. **40Pin 排针 5V**：双向，USB 不供电时可输入，USB 供电时可对外输出（限流 1A）
3. **8-24V 输入**：GH1.25-2P 自锁座（最大 1A）或背部大焊盘（最大 2A），经 DCDC 降为 5V

> 具备过压、过流、防倒灌保护。排针 5V 输入限流 2A。

---

## 七、在线查询工具与实时查询

### 官方在线引脚查询工具（可视化）
👉 **https://wiki.lckfb.com/storage/html/lushan-pinout/index.html**

支持标准版/Lite 版切换，点击引脚查看完整复用功能。

### CanMV 实时查询

```python
from machine import FPIOA
fpioa = FPIOA()
fpioa.help()          # 打印所有引脚当前配置
fpioa.help(11)        # 查单个引脚
```

### 设置引脚功能

```python
from machine import FPIOA
fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)   # 将 GPIO11 设为 UART2_TXD
fpioa.set_function(2, FPIOA.GPIO2, ie=1, oe=1, pu=0, pd=0, st=1, ds=7)
```

---

## 八、Lite 版（128MB）与标准版（1GB）引脚差异

两版共享相同 FPIOA 复用矩阵，但 40Pin 排针有 **7 处物理引脚差异**：

| 物理 Pin | Lite 版 GPIO | 标准版 GPIO |
|----------|:---:|:---:|
| Pin 12 | GPIO62 | GPIO23 |
| Pin 15 | GPIO52 | GPIO22 |
| Pin 22 | GPIO53 | GPIO25 |
| Pin 26 | GPIO63 | GPIO24 |
| Pin 32 | GPIO42 (UART3_TXD) | GPIO19 |
| Pin 33 | GPIO43 (UART3_RXD) | GPIO18 |
| Pin 35 | GPIO10 | GPIO17 |

其余 33 个物理引脚两版完全一致。

---

## 九、接线注意事项

1. **所有 GPIO 电平 3.3V**，切勿直接接 5V 信号，否则烧芯片
2. **ADC 仅 1.8V**，且不在 40Pin 排针上，需用 FPC 转接
3. **串口交叉接线**：K230 TXD → 外设 RXD，K230 RXD → 外设 TXD
4. **半双工总线**（如舵机单总线）：TX 串 1kΩ 电阻接信号线，RX 直连
5. **舵机/电机等大电流设备独立供电**，GND 必须与 K230 共地
6. **UART0 不可用**：被 RT-Smart 大核占用为调试串口
