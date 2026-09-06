---
name: "fashionstar-servo-k230"
description: "在庐山派K230(CanMV/MicroPython)上经UART直接控制FashionStar总线伺服舵机：FSUS协议帧、MicroPython驱动、双轴舵机云台视觉追踪、同步指令、保护参数与PID调试。当需要K230直控舵机、舵机云台追踪或排查舵机通信时调用。"
---

# FashionStar 总线舵机 — 庐山派 K230 直控 Skill

## 0. 适用场景与边界

**适用：**
- 在庐山派 K230-CanMV（MicroPython）上通过 UART **直接**控制 FashionStar UART/RS-485 总线伺服舵机（HP/HA/HX 等全系列）。
- 典型用法：K230 视觉识别 → 像素偏差 `dx,dy` → 直接驱动双轴舵机云台追踪（**不经过 STM32**）。
- 舵机通信排查（ping/角度回读/状态监控/丢包）、保护参数与 ID/波特率配置、云台 PID 调试。

**不适用：**
- PC（Windows/Ubuntu）C++ 开发 → 使用官方 C++ SDK，见 [云台/cpp-sdk](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk) 与 https://fashionstar.com.cn/wiki/sdk/servo/cpp-sdk/ 。
- STM32 等单片机 → C++ SDK 依赖 CSerialPort 不可用，需按本 Skill 第 2 章协议移植 C 版本（官方亦有 STM32 SDK）。

**协议权威来源：** 本 Skill 的协议帧格式从 [FashionStar_UartServoProtocol.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/src/FashionStar_UartServoProtocol.cpp) 逐行提炼。任何协议疑问以该源码 + 官网 wiki 为准。

---

## 1. 硬件接线

### 1.1 K230 UART 引脚（庐山派 40-Pin）

| UART | TX (物理Pin / GPIO) | RX (物理Pin / GPIO) | 备注 |
|------|---------------------|---------------------|------|
| UART1 | Pin 8 / GPIO 3 | Pin 10 / GPIO 4 | 备用 |
| **UART2** | **Pin 11 / GPIO 5** | **Pin 13 / GPIO 6** | 工程默认（K230 直控舵机总线） |
| UART3 | Pin 37 / GPIO 32 | Pin 40 / GPIO 33 | 扩展 |

> 当前工程 K230 **直控舵机**，舵机总线接 UART2（GPIO5/6），不再经过 STM32。

### 1.2 舵机总线接线（半双工）

FashionStar 总线舵机为 **半双工单信号线菊花链总线**（3 线：GND / VCC / 信号）：

```
K230 GPIO5(TX) ──[≈1kΩ]──┬──→ 舵机总线信号线（各舵机信号脚并联）
K230 GPIO6(RX) ──────────┘
K230 GND ──────────────── GND（必须共地）
舵机电源 ─── 独立电源（按型号，如 7.4V/12V），勿用 K230 供电
```

- TX 串 1kΩ 左右电阻再接信号线，避免舵机回复时与 K230 TX 争抢总线；RX 直连。
- **共地**是通信成功的前提；舵机电流大，必须独立供电，电源 GND 与 K230 GND 相连。
- K230 IO 为 3.3V，舵机总线为 TTL 电平，3.3V 高电平一般可被识别；若通信不稳定可加 3.3V/5V 电平转换。
- 多舵机靠 **舵机 ID** 区分（0~254，255 为广播地址，仅 ping 用），总线手拉手并联。
- **强烈建议先用 UC-01 USB 转接板 + 官方上位机验证每个舵机**（改 ID、测功能、写保护参数），再接 K230。

---

## 2. FSUS 通信协议精解（MicroPython 移植依据）

### 2.1 数据帧格式

- 字节序：**小端 Little-Endian**；多字节数值低字节在前。
- 请求帧头 `0x4C12`（线上：`0x12, 0x4C`）；响应帧头 `0x1C05`（线上：`0x05, 0x1C`）。
- `content_size < 255` 时长度占 1 字节（本协议所有常用指令均满足）。

```
请求: [0x12][0x4C][cmdId][size][content...][checksum]
响应: [0x05][0x1C][cmdId][size][content...][checksum]
```

**校验和**（1 字节）：帧头 2 字节 + cmdId + size + 全部 content 字节求和，取 `% 256`。

### 2.2 指令表（cmdId）

| cmdId | 指令 | content（请求） | 响应 content |
|---|---|---|---|
| 1 | PING 通信检测 | `[id]` | `[id]` |
| 3 | 读用户数据 | `[id, addr]` | `[id, addr, data...]` |
| 4 | 写用户数据 | `[id, addr, len, data...]` | `[id, addr, result]` |
| 8 | 单圈设角度 | `[id, 角度i16×10, 周期u16 ms, 功率u16 mW]`（7B） | 无 |
| 9 | 阻尼模式 | `[id, 功率u16 mW]` | 无 |
| 10 | 查询单圈角度 | `[id]` | `[id, 角度i16×10]` |
| 11 | 设角度(指定周期) | `[id, 角度i16, 周期u16, t_acc u16, t_dec u16, 功率u16]`（11B） | 无 |
| 12 | 设角度(指定转速) | `[id, 角度i16, 转速u16×10(0.1°/s), t_acc, t_dec, 功率]`（11B） | 无 |
| 13 | 多圈设角度 | `[id, 角度i32×10, 周期u32 ms, 功率u16]`（11B） | 无 |
| 16 | 查询多圈角度 | `[id]` | `[id, 角度i32×10]` |
| 17 | 复位多圈圈数 | `[id]` | 无 |
| 18 | 开始异步（缓存下一条指令） | 空 | 无 |
| 19 | 结束异步 | `[mode]`（0=执行缓存，1=取消） | 无 |
| 22 | 数据监控 | `[id]` | 16B，见 2.5 |
| 23 | 设置原点（当前位置清零） | `[id, 0]` | 无 |
| 24 | 控制模式停止 | `[id, mode\|0x10, 功率u16]` | 无 |
| 25 | 同步指令（多舵机同帧） | `[子cmd, 每舵机字节数, 数量, 各舵机数据...]` | 无/监控包 |

- 角度单位：线上为 **0.1°**（如 90.0° → 900）。单圈范围 ±180°（源码限幅 ±180），多圈 ±368640°。
- `周期 interval`：舵机在该时间（ms）内走完行程（梯形速度规划）；追踪时建议设为帧周期（30~50ms）。
- `功率 power`：功率上限 mW，**0 = 不限制**。
- cmd24 停止模式 mode：**0=停止后卸力失锁，1=保持锁力，2=进入阻尼**，线上发送 `mode | 0x10`。
- 同步指令（cmd25）：子 cmd 取值 8/11/12/13/14/15（对应单舵机指令），每舵机字段与单舵机 content 去掉 id 后相同；子 cmd 8 时每舵机 7B：`[id, 角度i16×10, 周期u16, 功率u16]`。双轴云台用 **cmd25 + 子cmd8 一帧驱动两个舵机**，最省总线带宽。

### 2.3 用户数据地址（cmd3/cmd4 读写）

| 地址 | 含义 | 长度/单位 |
|---|---|---|
| 1~5 | 只读：电压 mV / 电流 mA / 功率 mW / 温度ADC / 状态位 | u16 / u8 |
| 33 | 反馈与中断开关（0x00 可中断无反馈，默认） | u8 |
| 34 | 舵机 ID（0~254，255 广播） | u8 |
| 36 | 波特率：1=9600 2=19200 3=38400 4=57600 **5=115200(默认)** 6=250000 7=500000 | u8，即时生效 |
| 37 | 堵转保护模式（0=降功率，1=释放锁力） | u8 |
| 38 | 堵转功率上限 mW | u16 |
| 39/40 | 电压下限/上限 mV | u16 |
| 41 | 温度上限 ℃ | u16 |
| 42 | 功率上限 mW | u16 |
| 43 | 电流上限 mA | u16 |
| 44 | 加速度处理开关（1=梯形加减速，默认） | u8 |
| 46 | 上电锁力（0x00=释放默认，0x11=刹车） | u8 |
| 48 | 角度限制开关（1=开启，51/52 才生效） | u8 |
| 49/50 | 上电缓启动开关 / 缓启动时间 ms | u8/u16 |
| 51/52 | 角度上限/下限（0.1°） | u16 |
| 53 | 中位角度偏移（0.1°） | u16 |

> 写参数后建议**断电重启生效**；改 ID/波特率后主机端必须同步更改。批量改参优先用官方上位机。

### 2.4 状态位（status，bit 掩码）

| bit | 含义 | bit | 含义 |
|---|---|---|---|
| 0 | 指令执行中（完成清零） | 4 | 电压过低 |
| 1 | 指令错误 | 5 | 电流错误 |
| 2 | 堵转 | 6 | 功率错误 |
| 3 | 电压过高 | 7 | 温度错误 |

### 2.5 数据监控响应（cmd22，content 16 字节）

```
[0]id
[1..2] 电压 u16 mV      [3..4] 电流 i16 mA     [5..6] 功率 i16 mW
[7..8] 温度 ADC u16     [9] 状态位 u8
[10..13] 角度 i32 ×0.1°  [14..15] 圈数 i16
```

温度 ADC→℃（Steinhart-Hart，源码公式）：

```python
T = 1.0/(math.log(adc/(4096.0-adc))/3435.0 + 1.0/298.15) - 273.15
```

---

## 3. K230 MicroPython 驱动（完整可用）

> **当前落地方式（单文件部署）**：FSUS 驱动（`FSServoBus`/`FSServo`/`sync_set_angles`）已**内嵌**进 [k230/main.py](file:///c:/Users/12553/Desktop/视觉/复刻/k230/main.py)，部署只需把 `main.py` + `config.json` 拷到 SD 卡根目录即可，不再需要单独的 `fs_servo.py`。驱动类定义在 `main.py` 顶部（搜索 `class FSServoBus`），帧格式已用 mock 串口与 C++ SDK 逐字节校验。下方为完整代码清单（如需独立拆分可直接保存为 `fs_servo.py` 使用）。

```python
# fs_servo.py — FashionStar 总线舵机 K230(MicroPython) 直控驱动
# 协议依据: 云台/cpp-sdk/src/FashionStar_UartServoProtocol.cpp
from machine import UART, FPIOA
import time
import math

# ---- 指令常量 ----
CMD_PING=1; CMD_READ=3; CMD_WRITE=4; CMD_SET_ANGLE=8; CMD_DAMPING=9
CMD_QUERY_ANGLE=10; CMD_SET_BY_INTERVAL=11; CMD_SET_BY_VELOCITY=12
CMD_SET_MTURN=13; CMD_QUERY_MTURN=16; CMD_RESET_MTURN=17
CMD_BEGIN_ASYNC=18; CMD_END_ASYNC=19; CMD_MONITOR=22
CMD_ORIGIN=23; CMD_STOP=24; CMD_SYNC=25


class FSServoBus:
    """一条舵机总线 = 一个 UART"""
    def __init__(self, uart_id=2, baud=115200, tx_pin=5, rx_pin=6, timeout_ms=100):
        fp = FPIOA()
        fname_t = {1: FPIOA.UART1_TXD, 2: FPIOA.UART2_TXD, 3: FPIOA.UART3_TXD}[uart_id]
        fname_r = {1: FPIOA.UART1_RXD, 2: FPIOA.UART2_RXD, 3: FPIOA.UART3_RXD}[uart_id]
        fp.set_function(tx_pin, fname_t)
        fp.set_function(rx_pin, fname_r)
        self.uart = UART(uart_id, baudrate=baud)
        self.tmo = timeout_ms

    # ---- 低层帧收发 ----
    def _pack(self, cmd, content):
        n = len(content)
        f = bytearray(4 + n + 1)
        f[0] = 0x12; f[1] = 0x4C; f[2] = cmd; f[3] = n
        f[4:4+n] = content
        chk = 0x12 + 0x4C + cmd + n
        for b in content:
            chk += b
        f[4+n] = chk & 0xFF
        return f

    def _read_n(self, n):
        buf = bytearray()
        t0 = time.ticks_ms()
        while len(buf) < n:
            if time.ticks_diff(time.ticks_ms(), t0) > self.tmo:
                return None
            m = self.uart.any()
            if m:
                buf += self.uart.read(min(m, n - len(buf)))
        return buf

    def _txrx(self, cmd, content, wait_resp=True, retry=1):
        for _ in range(retry + 1):
            self.uart.read()  # 清空接收缓存
            self.uart.write(self._pack(cmd, content))
            if not wait_resp:
                return None
            hdr = self._read_n(4)            # 帧头2 + cmd + size
            if hdr is None or hdr[0] != 0x05 or hdr[1] != 0x1C:
                continue
            size = hdr[3]
            body = self._read_n(size + 1)    # content + checksum
            if body is None:
                continue
            cont = body[:size]
            chk = (0x05 + 0x1C + hdr[2] + size + sum(cont)) & 0xFF
            if chk == body[size]:
                return hdr[2], cont
        return None


class FSServo:
    def __init__(self, bus, sid):
        self.bus = bus
        self.id = sid
        self.cur_angle = 0.0
        # 两点标定: 真实角度 real -> 原始 raw = k*real + b
        self.k = 1.0
        self.b = 0.0

    def ping(self):
        r = self.bus._txrx(CMD_PING, bytes([self.id]))
        return r is not None and r[1][0] == self.id

    # ---- 单圈角度 ----
    def set_angle(self, angle_deg, interval_ms=0, power=0):
        a = int(max(-180.0, min(180.0, angle_deg)) * 10)
        self.bus._txrx(CMD_SET_ANGLE, bytes([
            self.id, a & 0xFF, (a >> 8) & 0xFF,
            interval_ms & 0xFF, (interval_ms >> 8) & 0xFF,
            power & 0xFF, (power >> 8) & 0xFF]), wait_resp=False)

    def set_angle_by_interval(self, angle_deg, interval_ms, t_acc=100, t_dec=100, power=0):
        a = int(angle_deg * 10)
        self.bus._txrx(CMD_SET_BY_INTERVAL, bytes([
            self.id, a & 0xFF, (a >> 8) & 0xFF,
            interval_ms & 0xFF, (interval_ms >> 8) & 0xFF,
            t_acc & 0xFF, (t_acc >> 8) & 0xFF,
            t_dec & 0xFF, (t_dec >> 8) & 0xFF,
            power & 0xFF, (power >> 8) & 0xFF]), wait_resp=False)

    def set_angle_by_velocity(self, angle_deg, velocity_dps, t_acc=100, t_dec=100, power=0):
        a = int(angle_deg * 10); v = int(velocity_dps * 10)  # 线上 0.1°/s
        self.bus._txrx(CMD_SET_BY_VELOCITY, bytes([
            self.id, a & 0xFF, (a >> 8) & 0xFF,
            v & 0xFF, (v >> 8) & 0xFF,
            t_acc & 0xFF, (t_acc >> 8) & 0xFF,
            t_dec & 0xFF, (t_dec >> 8) & 0xFF,
            power & 0xFF, (power >> 8) & 0xFF]), wait_resp=False)

    def query_angle(self):
        r = self.bus._txrx(CMD_QUERY_ANGLE, bytes([self.id]), retry=2)
        if r is None:
            return None
        c = r[1]
        a = c[1] | (c[2] << 8)
        if a >= 32768:
            a -= 65536
        self.cur_angle = a * 0.1
        return self.cur_angle

    # ---- 多圈 ----
    def set_angle_mturn(self, angle_deg, interval_ms=0, power=0):
        a = int(angle_deg * 10)
        self.bus._txrx(CMD_SET_MTURN, bytes([
            self.id,
            a & 0xFF, (a >> 8) & 0xFF, (a >> 16) & 0xFF, (a >> 24) & 0xFF,
            interval_ms & 0xFF, (interval_ms >> 8) & 0xFF,
            (interval_ms >> 16) & 0xFF, (interval_ms >> 24) & 0xFF,
            power & 0xFF, (power >> 8) & 0xFF]), wait_resp=False)

    def reset_mturn(self):
        self.bus._txrx(CMD_RESET_MTURN, bytes([self.id]), wait_resp=False)

    # ---- 模式 ----
    def damping(self, power=500):
        """阻尼模式: 舵机以 power(mW) 抵抗外力, 可安全手动掰动"""
        self.bus._txrx(CMD_DAMPING, bytes([
            self.id, power & 0xFF, (power >> 8) & 0xFF]), wait_resp=False)

    def stop(self, mode=2, power=500):
        """停止: 0=卸力失锁 1=保持锁力 2=阻尼(默认, 安全)"""
        self.bus._txrx(CMD_STOP, bytes([
            self.id, (mode | 0x10) & 0xFF,
            power & 0xFF, (power >> 8) & 0xFF]), wait_resp=False)

    def set_origin(self):
        """当前机械位置记为零点"""
        self.bus._txrx(CMD_ORIGIN, bytes([self.id, 0]), wait_resp=False)

    # ---- 数据监控 ----
    def monitor(self):
        r = self.bus._txrx(CMD_MONITOR, bytes([self.id]), retry=2)
        if r is None:
            return None
        c = r[1]
        def u16(o): return c[o] | (c[o+1] << 8)
        def i16(o):
            v = u16(o); return v - 65536 if v >= 32768 else v
        def i32(o):
            v = c[o] | (c[o+1] << 8) | (c[o+2] << 16) | (c[o+3] << 24)
            return v - 4294967296 if v >= 2147483648 else v
        adc = u16(7)
        temp = -999.0
        if 0 < adc < 4096:
            try:
                temp = 1.0/(math.log(adc/(4096.0-adc))/3435.0 + 1.0/298.15) - 273.15
            except Exception:
                pass
        return {"id": c[0], "voltage_mV": u16(1), "current_mA": i16(3),
                "power_mW": i16(5), "temp_C": round(temp, 1),
                "status": c[9], "angle": i32(10) * 0.1, "circle": i16(14)}

    # ---- 参数读写 ----
    def read_param(self, addr):
        r = self.bus._txrx(CMD_READ, bytes([self.id, addr]), retry=2)
        return None if r is None else r[1][2:]   # 去掉 id, addr

    def write_param(self, addr, data):
        self.bus._txrx(CMD_WRITE,
                       bytes([self.id, addr, len(data)]) + bytes(data),
                       wait_resp=False)

    def set_id(self, new_id):
        self.write_param(34, [new_id]); self.id = new_id

    def set_baud(self, baud_code):
        """5=115200(默认) 6=250000 7=500000 ... 写入后即时生效, 主机需同步改波特率"""
        self.write_param(36, [baud_code])

    # ---- 标定 ----
    def calibration(self, raw_a, real_a, raw_b, real_b):
        """两点标定: raw = k*real + b (raw=舵机读数, real=机械真实角)"""
        self.k = (raw_a - raw_b) / (real_a - real_b)
        self.b = raw_a - self.k * real_a

    def set_real_angle(self, real_angle_deg, interval_ms=0, power=0):
        self.set_angle(self.k * real_angle_deg + self.b, interval_ms, power)


def sync_set_angles(bus, targets, interval_ms=0, power=0):
    """同步指令(单圈): 一帧驱动多个舵机同时动作。
    targets: [(id, angle_deg), ...] 数量 ≤ 5"""
    n = len(targets)
    c = bytearray([8, 7, n])   # 子cmd=8(设角度), 每舵机7B, 数量
    for sid, ang in targets:
        a = int(max(-180.0, min(180.0, ang)) * 10)
        c += bytes([sid, a & 0xFF, (a >> 8) & 0xFF,
                    interval_ms & 0xFF, (interval_ms >> 8) & 0xFF,
                    power & 0xFF, (power >> 8) & 0xFF])
    bus._txrx(CMD_SYNC, c, wait_resp=False)
```

### 3.1 最小自检（先跑这个）

```python
# 把第3章的驱动代码另存为 fs_servo.py 后运行此自检
from fs_servo import FSServoBus, FSServo

bus = FSServoBus(uart_id=2, baud=115200, tx_pin=5, rx_pin=6)
s0 = FSServo(bus, 0)

print("ping id0:", s0.ping())
s0.set_angle(0)             # 回中
import time; time.sleep(1)
s0.set_angle(90, interval_ms=1000)   # 1s 内转到 90°
time.sleep(1.2)
print("angle:", s0.query_angle())
print("monitor:", s0.monitor())
s0.damping(500)             # 阻尼, 可用手掰
```

### 3.2 MicroPython 性能注意（与本工程 main.py 热路径规范一致）

- 追踪主循环里**每帧只发一帧同步指令**（`sync_set_angles`），避免逐舵机轮询。
- 角度查询/监控是"发一帧等一帧"的阻塞操作（超时 100ms），**不要放进视觉主循环**；需要监控时低频（如 1~2Hz）或在独立状态下查询。
- 帧对象已用 `bytearray` 预分配式构造；不要在循环内做字符串格式化/字典分配。
- `set_angle` 类写指令 `wait_resp=False` 不等回复，速度最快；关键指令（ping/query）带 `retry`。

---

## 4. 双轴舵机云台视觉追踪框架

### 4.1 系统形态

```
K230-CanMV
├── 摄像头 → 视觉检测 → dx,dy 像素偏差（复用现有 main.py 检测链）
└── UART2 ──→ 舵机总线（Yaw 舵机 ID=0 + Pitch 舵机 ID=1，同步指令）
```

视觉外环为 **P 控制**：每帧把像素偏差换算成角度增量，累加到目标角，用同步指令下发。舵机内部位置环（第 6 章 PID）保证到位。

### 4.2 config.json 增加 `servo` 段

```json
"servo": {
  "enabled": true,
  "mode": "track",
  "port": 2, "baud": 115200, "tx_pin": 5, "rx_pin": 6,
  "yaw_id": 1, "pitch_id": 0,
  "yaw_vel_scale": 0.01, "pitch_k": 0.02,
  "_scale": "每帧角度增量系数: 角度 += scale × 偏移像素。从0.01起调，慢则加大、振荡则减小",
  "yaw_reverse": false, "pitch_reverse": true,
  "yaw_min": -360, "yaw_max": 360,
  "pitch_min": -30, "pitch_max": 30,
  "interval_ms": 40, "power": 0,
  "deadband_px": 5,
  "lost_hold": true,
  "center_on_boot": true
}
```

> `center_on_boot: true` 时上电用 2s 缓慢回中（要求上电机械位置大致居中）。启动时会自动 ping 检测 yaw/pitch 舵机是否在线，并读取地址 48/51/52 解除舵机内部角度限制。

### 4.3 与 main.py 集成（已内嵌为单文件）

> **当前实现**：`servo_send` 已直接内嵌进 [main.py](file:///c:/Users/12553/Desktop/视觉/复刻/k230/main.py)，配置项在 [config.json](file:///c:/Users/12553/Desktop/视觉/复刻/k230/config.json) 的 `servo` 段。`main.py` 仅保留**舵机控制**输出，已删除 STM32 串口转发、卡尔曼滤波等无关逻辑。

实现要点（与 main.py 内一致）：

- 偏移统一以**物体中心**为准：CV 模式用矩形中心 `quad_center(box)`，YOLO 模式用识别框中心 `(x+w//2, y+h//2)`，计算 `dx = cx - CENTER_X, dy = cy - CENTER_Y`。
- 控制为**增量式**：每帧 `_yaw_ang += YAW_REV * YAW_VEL_SCALE * dx`，`_pitch_ang += PITCH_REV * PITCH_K * dy`；偏差 >50px 时按 `(abs-50)/80` 加速。
- 死区内不动作并 `stop(1)` 锁力；丢靶按 `lost_hold` 决定保持或阻尼。
- 启动时 `center_on_boot` 上电 2s 缓慢回中，并自动读取地址 48/51/52 解除舵机内部角度限制。

下方代码为实现参考（main.py 中已直接包含，无需外部 import）：

```python
# 驱动已内嵌, 无需 from fs_servo import ...

sv = _get(CFG, "servo", {})
SERVO_ENABLED = sv.get("enabled", False)

if SERVO_ENABLED:
    try:
        _sbus = FSServoBus(uart_id=sv.get("port", 2), baud=sv.get("baud", 115200),
                           tx_pin=sv.get("tx_pin", 5), rx_pin=sv.get("rx_pin", 6))
        _yaw = FSServo(_sbus, sv.get("yaw_id", 1))
        _pitch = FSServo(_sbus, sv.get("pitch_id", 0))
        YAW_K = sv.get("yaw_vel_scale", 0.01); PITCH_K = sv.get("pitch_k", 0.02)
        YAW_REV = -1.0 if sv.get("yaw_reverse", False) else 1.0
        PITCH_REV = -1.0 if sv.get("pitch_reverse", False) else 1.0
        YAW_MIN, YAW_MAX = sv.get("yaw_min", -360), sv.get("yaw_max", 360)
        PITCH_MIN, PITCH_MAX = sv.get("pitch_min", -30), sv.get("pitch_max", 30)
        S_INTERVAL = sv.get("interval_ms", 40); S_POWER = sv.get("power", 0)
        DEADBAND = sv.get("deadband_px", 5); LOST_HOLD = sv.get("lost_hold", True)
        _yaw_ang = 0.0; _pitch_ang = 0.0

        def _clamp(v, lo, hi):
            return lo if v < lo else (hi if v > hi else v)

        def servo_send(dx, dy, dist, status):
            global _yaw_ang, _pitch_ang
            if status == "lost" or status == 404:
                if not LOST_HOLD:
                    _yaw.stop(2); _pitch.stop(2)   # 丢靶→阻尼保安全
                return
            if abs(dx) > DEADBAND:
                _yaw_ang = _clamp(_yaw_ang - YAW_REV * YAW_K * dx, YAW_MIN, YAW_MAX)
            if abs(dy) > DEADBAND:
                _pitch_ang = _clamp(_pitch_ang - PITCH_REV * PITCH_K * dy, PITCH_MIN, PITCH_MAX)
            sync_set_angles(_sbus,
                            [(_yaw.id, _yaw_ang), (_pitch.id, _pitch_ang)],
                            S_INTERVAL, S_POWER)
        print("servo: yaw_id=%d pitch_id=%d k=%.3f/%.3f" %
              (_yaw.id, _pitch.id, YAW_K, PITCH_K))
    except Exception as e:
        print("servo init failed:", e)
        servo_send = lambda dx, dy, d, s: None
else:
    servo_send = lambda dx, dy, d, s: None
```

主循环中调用 `servo_send(int(fdx), int(fdy), dist, "track")`（追踪中）/ 丢失时 `servo_send(0,0,0,"lost")`。`fdx/fdy` 为物体中心相对屏幕中心的像素偏移。

### 4.4 调参要点

| 现象 | 调整 |
|---|---|
| 追踪太慢、跟不上 | 加大 `yaw_vel_scale/pitch_k`；适当加大 `interval_ms` 内转速或设 power=0 |
| 振荡/过冲/来回摆 | 减小 scale；增大 `deadband_px` |
| 单方向偏差越跟越远 | 翻转对应 `*_reverse` |
| 到位后小幅抖动 | 增大死区（5~8px）；舵机内部 PID 降 Kp（第 6 章） |
| 动作卡顿 | 确保每帧只发 1 帧同步指令；视觉跳帧检测；总线降干扰 |
| 丢靶后乱转 | `lost_hold=true` 保持最后位置，或丢靶即 `stop(2)` 阻尼 |

- `yaw_vel_scale/pitch_k` 经验起点：`0.01~0.03` 度/像素（640×480 画面、舵机直接负载）。
- `interval_ms` 设为视觉帧周期（如 30~50ms），舵机持续滚动更新目标角，轨迹最平滑。
- 方向约定：`dx>0` 目标在画面右侧 → 云台应向右转；实际方向取决于舵机安装朝向，用 reverse 开关校正。

---

## 5. 舵机两点标定（机械角度校正）

舵机回读的原始角度（raw）与机械真实角度（real）可能存在方向/偏移差异。在两个已知机械角位置记录回读值：

```python
# 样本: real=+90° 时 raw=-86.2 ; real=-90° 时 raw=91.9（以实测为准）
s0.calibration(raw_a=-86.2, real_a=90, raw_b=91.9, real_b=-90)
s0.set_real_angle(90)     # 自动换算为原始角度下发
```

- 公式：`raw = k*real + b`，`k=(rawA-rawB)/(realA-realB)`，`b=rawA-k*realA`。
- 标定后用 `set_real_angle()` / 查询值按 k,b 反算真实角。
- 机械安装偏差也可用写参数 **地址53 中位偏移（0.1°）** 或 `set_origin()` 清零处理。

---

## 6. 舵机保护参数、上位机与云台 PID

### 6.1 保护参数（K230 写入示例）

```python
s = FSServo(bus, 0)
s.write_param(48, [1])                       # 开启角度限制
s.write_param(51, [0x40, 0x06])              # 角度上限 160.0° = 1600 = 0x0640
s.write_param(52, [0xC0, 0xF9])              # 角度下限 -160.0° = -1600 = 0xF9C0
s.write_param(41, [80, 0])                   # 温度上限 80℃
s.write_param(42, [0x70, 0x17])              # 功率上限 6000mW = 0x1770
s.write_param(49, [1]); s.write_param(50, [0xB8, 0x0B])  # 缓启动开, 3000ms
```

> 多字节参数均小端编码；写后**断电重启生效**。保护值/ID/波特率批量配置推荐用官方上位机（FashionStar UART 总线舵机上位机，下载：https://fashionrobo.com/downloadcenter/ ），先记录原值再修改。

### 6.2 云台应用舵机内部 PID 推荐值（出厂已调好，仅云台负载变化时修改）

| 型号 | Kp | Ki | Kd |（出厂默认 Kp/Ki/Kd） |
|---|---|---|---|---|
| HA/RA8-U25 | 1750 | 1 | 7005 | 800/1/5000 |
| HA/RA8-U25-M | 800 | 0 | 5000 | 800/0/5000 |
| HA/RA8-U25H-M | 400 | 0 | 2000 | 400/0/2300 |
| HP/RP8-U45-M | 320 | 0 | 2500 | 320/0/1000 |
| HP/RP8-U45H-M | 300 | 0 | 700 | 300/0/700 |
| HX/RX8-U28H-M | 800 | 0 | 80 | 600/0/50 |

**调节步骤（上位机写入，断电重启生效；务必 Kp=Hold Kp、Kd=Hold Kd）：**
1. Ki、Kd 先置 0，逐步加大 Kp 至轻微振荡，再回落到不振荡，取该值的 60%~70%。
2. Ki 默认 0；PD 精度不够再设 1；出现振荡/抖动立即取消积分。
3. 固定 Kp 后逐步加大 Kd 至振荡再回落，取不振荡值的约 30%。
4. 先空载、再带载、最后微调；追求稳定用 PD，追求响应用 PI。

> 注意区分两层控制：**舵机内部 PID**（位置环，上位机调）决定单舵机响应特性；**K230 视觉外环**（第 4 章，P 控制 `yaw_vel_scale/pitch_k`）决定追踪行为。云台"丝滑运行"用出厂默认内部 PID 即可，追踪效果主要调外环 scale 与死区。

---

## 7. 排障清单

| 现象 | 排查 |
|---|---|
| ping 离线 | ① 舵机独立供电且与 K230 共地 ② TX/RX 接线（TX 串电阻接信号、RX 直连）③ 波特率 115200 与舵机一致 ④ ID 是否正确（可广播 255 扫描）⑤ FPIOA 功能号与 UART 号匹配 |
| 时通时断/校验错误多 | 供电不足/线太长/干扰：加粗电源线、缩短信号线、降低波特率（如 57600）、加入 retry；参考官网"指令丢包问题及解决方法" |
| 在线但不转 | 检查阻尼/停止模式是否未退出、功率上限（地址42）、角度限位（48/51/52）、堵转保护 bit2、`set_angle` 的 interval/power |
| 角度值乱/方向反 | 单圈/多圈指令混用；用 `calibration()` 标定或 `set_origin()` 清零；翻转 reverse |
| 温度明显异常 | 监控返回的是 ADC，必须用第 2.5 节公式换算℃ |
| 改 ID/波特率后失联 | 新参数断电重启生效；主机端同步更新；广播 ping(ID=255) 找回 |
| K230 报 tx not configured | FPIOA 未映射或引脚号错：UART2=GPIO5(TX)/GPIO6(RX) |
| 视觉循环帧率暴跌 | 阻塞查询（query/monitor）移出主循环；每帧只发一帧同步写指令 |

参考 wiki：
- 舵机未检测/无法响应：https://fashionstar.com.cn/wiki/documents/servo/troubleshooting-not-detected-or-no-response/
- 保护参数设置：https://fashionstar.com.cn/wiki/documents/servo/setting-protection-parameters/
- 指令丢包：https://fashionstar.com.cn/wiki/documents/servo/command-packet-loss/
- 云台 PID 调节：https://fashionstar.com.cn/wiki/documents/servo/pid-tuning-guide/

---

## 8. C++ SDK 对照（本地资料速查）

协议/用法疑问时对照官方 C++ SDK（PC 端可直接编译运行验证，再把行为移植到 MicroPython）：

| 功能 | C++ API（FSUS_Servo） | 本 Skill MicroPython | 例程 |
|---|---|---|---|
| 通信检测 | `ping()` | `FSServo.ping()` | [servo_ping.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_ping/servo_ping.cpp) |
| 单圈设角度 | `setRawAngle/setAngle` | `set_angle()` | [servo_set_angle.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_set_angle/servo_set_angle.cpp) |
| 角度回读 | `queryRawAngle()` | `query_angle()` | [servo_query_angle.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_query_angle/servo_query_angle.cpp) |
| 多圈控制 | `setRawAngleMTurn*` | `set_angle_mturn()` | [servo_set_angle_mturn.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_set_angle_mturn/servo_set_angle_mturn.cpp) |
| 阻尼 | `setDamping(power)` | `damping(power)` | [servo_damping.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_damping/servo_damping.cpp) |
| 同步指令 | `setSyncRawAngle(count, arr)` | `sync_set_angles()` | [servo_synccommand_mode.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_synccommand_mode/servo_synccommand_mode.cpp) |
| 数据监控 | `querymonitor()` | `monitor()` | [servo_query_monitor.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_query_monitor/servo_query_monitor.cpp) |
| 状态读取 | `queryVoltage/Current/Power/Temperature/Status` | `read_param(1..5)` / `monitor()` | [servo_data_read.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_data_read/servo_data_read.cpp) |
| 原点设置 | `SetOriginPoint()` | `set_origin()` | [servo_set_origin_point.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_set_origin_point/servo_set_origin_point.cpp) |
| 停止模式 | `SetStopOnControlMode(mode,power)` | `stop(mode,power)` | [servo_stop_oncontrolmode.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_stop_oncontrolmode/servo_stop_oncontrolmode.cpp) |
| 异步缓存 | `SetBeginAsync/SetEndAsync` | `write_param`/裸指令（追踪场景用同步指令即可） | [servo_begin_end_async.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_begin_end_async/servo_begin_end_async.cpp) |
| 两点标定 | `calibration(rawA,realA,rawB,realB)` | `calibration(...)` + `set_real_angle()` | [servo_calibration.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/example/servo_calibration/servo_calibration.cpp) |

协议常量与结构体：[FashionStar_UartServoProtocol.h](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/include/FashionStar_UartServoProtocol.h)；帧打包/解析权威实现：[FashionStar_UartServoProtocol.cpp](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk/src/FashionStar_UartServoProtocol.cpp)；PDF 手册见 [云台/cpp-sdk](file:///c:/Users/12553/Desktop/视觉/复刻/云台/cpp-sdk)。

> 已知 SDK 细节：C++ SDK 同步"指定转速"模式（子cmd12）发送转速字段时未 ×10（与单舵机指令不一致，疑似 bug）；本驱动统一按协议单位 0.1°/s（×10）实现，跨端混用时以实测转速为准。
