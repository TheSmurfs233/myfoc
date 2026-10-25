# 主机测试台（Host Test Harness）

在 **Windows 主机上直接跑固件逻辑**，不需要 STM32 硬件、不需要 WSL、不需要烧录。
用假的 HAL 桩替换外设，把 `User/` 下的纯逻辑模块编成 x86 程序跑单元测试。

---

## 1. 一分钟上手

```bash
cd D:/Embedded/STM32/dsh_foc
bash test/host/build.sh
```

输出示例：

```
===== myfoc host tests =====
[ring buffer]
  ringbuf: basic push/pop                   ok
  ...
============================
checks: 408   failed: 0
RESULT: PASS
```

- 返回码 `0` = 全部通过，`1` = 有失败
- 失败时自动打印 `文件:行号 断言表达式`，串口类测试还会 dump 出实际收到的字节

**编译器**：用的是本机 Vitis_HLS 自带的 msys2 mingw64 gcc
（`D:/Xilinx/Vitis_HLS/2022.2/tps/win64/msys64/mingw64/bin/gcc.exe`，gcc 6.2.0）。
脚本里已写死这个路径，换机器改 `build.sh` 顶部的 `GCC_BIN` 即可。

---

## 2. 目录结构

```
test/host/
  build.sh              编译 + 运行（一条命令）
  test_main.c           所有测试用例
  fake/
    stm32g4xx_hal.h     HAL 桩：GPIO / DMA / UART / ADC / DWT 的最小类型与函数
    tim.h               TIM1 寄存器模型（CCR/CCER/BDTR/EGR），给 bsp_pwm 用
    adc.h               影子 adc.h，只提供 hadc2
    main.h              影子头文件，覆盖 Core/Inc/main.h（提供 LED/KEY 引脚宏）
    usart.h             影子头文件，提供 huart1 / USART1
    fake_api.h          测试钩子（时间、按键、TX 捕获、RX 注入）
    hal_stub.c          钩子的实现 + 假外设行为
```

**为什么不改生产代码**：全部通过 `-I test/host/fake` 让假头文件**先于**真实头文件被找到，
零侵入。唯一的耦合点是 `bsp_uart.c` 里 `extern DMA_HandleTypeDef hdma_usart1_rx;`，
桩里定义了同名对象即可。

---

## 3. 假外设怎么模拟真实行为

| 外设 | 桩的行为 | 对应真实器件 |
|---|---|---|
| `HAL_GetTick()` | 返回测试可控的 `s_tick` | SysTick 1ms |
| `HAL_GPIO_WritePin` | 记录 PF1 电平到 `s_led_level` | LED 引脚 |
| `HAL_GPIO_ReadPin` | KEY 引脚返回 `fake_key_set()` 设的电平 | 外部 10k 上拉 + 按键 |
| `HAL_UART_Transmit_DMA` | 把数据拷进捕获缓冲，**立即**回调 `TxCplt`（模拟瞬时完成） | DMA1_CH2 |
| `HAL_DMA_Init` | 把 `Init.Mode` 写进 `Instance->CCR` | DMA 通道配置 |
| `HAL_UARTEx_ReceiveToIdle_DMA` | 记下接收缓冲指针与长度 | DMA1_CH1 环形接收 |

### 3.1 RX 注入为什么必须模拟"每次回绕一次 TC 事件"

真实 HAL 在环形模式下：
- 缓冲区**写满一圈** → 触发 `HAL_UART_RXEVENT_TC`，回调的 `Size = RxXferSize`
- 线路**空闲** → 触发 `HAL_UART_RXEVENT_IDLE`，回调的 `Size = 当前写位置`

`fake_rx_feed()` 严格照这个来：每走满 128 字节就发一次 TC，结尾再发一次 IDLE。
**如果不这么做，就测不出固件里那个"连续两圈数据被整圈丢弃"的 bug**（见下方第 5 节）。

---

## 4. 覆盖范围（S1~S3，278 条断言）

| 模块 | 测了什么 |
|---|---|
| `mid_ringbuf` | 基本收发、满截断、回绕、peek 在边界截断、非 2 的幂拒绝、**20 万次随机操作对拍参考模型** |
| `mid_log` | `%d %i %u %x %X %s %c %f %%`、负数、`INT32_MIN`、宽度/补零/左对齐、`%f` 两位小数、精度修饰被吞、未知格式符透传、128 字节截断、空格式、**每条日志统一加 `[log] ` 前缀** |
| `mid_vofa` | 1/3/6 通道 **FireWater 纯 CSV 逐字节**比对（`1.500\n`、`1.000,-1.000,0.000\n`、`\n` 结尾、帧长有界）、非法通道数拒绝 |
| `bsp_uart` | TX 64 字节分块（200B → 64/64/64/8 共 4 次）、环容量上限 4095、`HAL_BUSY` 重试不丢数据、**TX 看门狗从丢失的完成事件中恢复**（不发 `TxCplt` → 超时 `DBG_TX_STUCK_MS` → abort + 继续）、RX 注入、**160B 穿过 128B 缓冲的回绕**、RX 溢出计数 |
| `bsp_gpio` | LED 低有效极性（PF1 低 = 亮）、20ms 消抖窗口内不触发、超过窗口只触发一次、长按不重复、松开事件 |
| `bsp_pwm` | `duty→CCR` 按 PWM2 反比映射（0.1→3825、0.5→2125、0.9→425）、0%/100% 与 NaN 钳到 `PWM_CCR_MIN/MAX`、`CCR→duty` 往返、`init` 后 MOE=0/CCxE=0/CCR=50%/CCR4 不被覆盖/计数器已启动、`enable` 置 MOE+全部 CCxE、`disable` 只清 MOE 与 CCxE（duty 保留）、读回的是钳位后的实际占空比 |
| `mid_current` | 256 帧零漂校准（差一帧不 ready）、均值取整、`(raw−off)×0.645mA` 正负方向、轨到轨饱和标记、重新校准回到未就绪 |
| `bsp_prof` | 显式周期数的 min/avg/max、空槽读 0、enable/disable、reset、`now()`/`add()` 用假 DWT 对拍 |
| `bsp_adc` | JEOC 回调锁存 JDR1~3、帧计数、OVR 计数并清除 |
| `app_main` | 模式范围校验、未选模式拒绝启动、运行中禁止切模式、波形速率钳位（0→1，3000→500）、**1 秒 @500Hz 恰好 500 帧、drop=0** |
| `app_cmd` | help/ver/stat、mode-start-stop 全流程、各种错误路径、未知命令、LED 控制、echo、退格编辑、超长行、空行忽略、一次收到多条命令 |

---

## 5. 这套测试实际抓到的 bug

| # | Bug | 现象 | 修法 |
|---|---|---|---|
| 1 | `ring_pop()` 跨回绕读不满 | `ring_pop(buf, 16)` 明明有 6 字节可用，只返回 4（peek 到边界就停） | 改成循环 peek+drop，直到读满或读空 |
| 2 | **UART 环形接收丢整圈数据** | 连续高速接收时，两次"缓冲区满"事件报告的 `Size` 都是 128，固件里 `Size == last` 早退直接返回，**整圈 128 字节丢失** | 改用 `huart->RxEventType` 区分 `TC`（整圈满）与 `IDLE`（位置更新）；TC 时搬 `last..END` 并把位置归零 |

第 2 条只在**持续不间断接收**时出现，用串口助手手敲命令永远测不出来 —— 这就是主机仿真的价值。

---

## 6. 后续阶段怎么往里加

### S4/S6：算法层（最有价值的部分）

算法层按设计**不依赖 HAL**（`Alg/` 只依赖 `Cfg/`），所以 S4 起只需：

```c
/* test_main.c */
#include "alg_clarke.h"
#include "alg_svpwm.h"

static void test_clarke(void)
{
    section("clarke: unity amplitude");
    T(fabsf(c.ialpha - 1.0f) < 1e-6f);
    ...
}
```

建议的用例：
- **Clarke/Park**：注入已知三相电流（含不平衡、含零序），对拍手算值；θ=0/90/180/270 四个特值
- **SVPWM**：给 `(vα,vβ)` 扫描整圈 360°×N 点，检查
  ①每相占空比在 `[CCR_MIN, CCR_MAX]` 内 ②三相对称 ③`max-min ≤ 1.0` ④线性区边界 `|V|=Vdc/√3`
- **PI 调节器**：单位阶跃响应，检查上升时间/超调/抗饱和（积分不 windup）
- **全链路**：给一个虚拟电机模型（RL + 反电动势 + 负载），闭环跑几万个周期，
  看 `iq` 是否跟随、`id` 是否收敛到 0。**这一步能在上电前把 90% 的 FOC bug 拦住。**

### 加新用例的步骤

1. 在 `test_main.c` 里写 `static void test_xxx(void)`，用 `section()` / `T()` 断言
2. 在 `main()` 里加一行调用
3. `bash test/host/build.sh`

---

## 6.5 台架验收脚本（`test/accept/`）

真机串口自动化，需要板子、烧录好的固件和空闲串口：

| 脚本 | 覆盖 | 断言数 |
|---|---|---|
| `s1_accept.py` | help(23 行)/ver/stat(13 行)、FireWater 禁冒号、按键/LED、状态机、`rate`、7 通道波形、丢帧 | 64 |
| `s2_accept.py` | PWM 命令、占空比钳位、启停、封波、行数回归 | 25 |
| `s3_accept.py` | 校准生命周期（erase→start→live→gen）、帧率、`pwm` 保护、DWT 时序画像 | 16 |

```bash
python test/accept/s3_accept.py COM7 1000000
```

> 帧率用**板子自己的 `uptime`** 计算（`stat` 里 `frames` 与 `uptime` 同一时刻采样），
> 不要用 PC 侧时间戳——命令往返 0.2s 会带来 4~8% 误差（踩过）。

主机侧目录里另有 `fake/bsp_flash_fake.c`：主机无法映射 `0x0801F800`，
所以链接时用它替换真实的 `bsp_flash.c`，**生产代码里不留任何测试分支**。

---

## 7. 注意事项

- **不要**把 `test/host/fake/**` 加进 Keil 工程（会和真实 HAL 冲突）
- 生产代码里**不允许**出现 `#ifdef HOST_TEST` 之类的测试专用分支；
  需要观测的点一律通过"可注入的输入 + 可捕获的输出"来实现（按键电平、串口收发）
- 主机测试**不能**验证：真实时序、中断延迟、ADC 采样时刻、PWM 死区、
  任何与模拟电路相关的行为。这些必须上台架用示波器看。
