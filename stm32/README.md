# STM32F407 云台端 — 编译说明

## 编译前准备

1. 从原项目 `Simple_Tracking_Device-main/MIAOZHUN/` 复制以下 HAL 生成文件到 `Core/`:
   - `Core/Inc/main.h`
   - `Core/Inc/stm32f4xx_hal_conf.h`
   - `Core/Inc/stm32f4xx_it.h`
   - `Core/Inc/usart.h`
   - `Core/Inc/dma.h`
   - `Core/Inc/gpio.h`
   - `Core/Inc/frame_parser.h`
   - `Core/Src/stm32f4xx_hal_msp.c`
   - `Core/Src/stm32f4xx_it.c`
   - `Core/Src/system_stm32f4xx.c`
   - `Core/Src/usart.c`
   - `Core/Src/dma.c`
   - `Core/Src/gpio.c`
   - `Core/Src/frame_parser.c`

2. 将 `stm32/User/` 下的文件复制到对应位置:
   ```
   stm32/User/main.c     → Core/Src/main.c     (替换原版)
   stm32/User/pid.c      → Core/Src/pid.c      (新增)
   stm32/User/pid.h      → Core/Inc/pid.h      (新增)
   stm32/User/DATOU.c    → Core/Src/DATOU.c    (保留原版)
   stm32/User/DATOU.h    → Core/Inc/DATOU.h    (保留原版)
   stm32/User/frame.c    → Core/Src/frame.c    (保留原版)
   stm32/User/frame.h    → Core/Inc/frame.h    (保留原版)
   stm32/User/Key.c      → Core/Src/Key.c      (保留原版)
   stm32/User/Key.h      → Core/Inc/Key.h      (保留原版)
   ```

3. 在 STM32CubeIDE 中添加 `pid.c` 到编译源文件列表

4. 确保 GPIO 初始化中配置了:
   - PB9: 输出 (电机使能)
   - PB8: 输出 (激光控制, 可选)

5. 编译 → 下载

## 与原项目的区别

原项目使用"按键触发→发0xA1→收到0x88→转固定角度"的**开环**控制。
复刻版改为"串口中断接收dx,dy→PID计算角度→连续调节电机"的**闭环**控制。
