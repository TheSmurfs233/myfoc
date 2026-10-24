# 子框架 · 按键与 LED（BSP 人机接口）

---

## 1. 职责边界

- **管**：LED 亮灭/翻转、按键消抖、按键边沿事件生成。
- **不管**：按键按下后干什么（那是 `app_main.c` 的 `key_action()`）。

---

## 2. 接口

```c
/* User/Bsp/bsp_gpio.h */
void    bsp_gpio_init(void);          /* PA0 开内部上拉；LED 上电熄灭 */

void    bsp_led_write(uint8_t on);    /* 1=亮（内部处理低有效） */
void    bsp_led_toggle(void);
uint8_t bsp_led_is_on(void);

void    bsp_key_poll(uint32_t now_ms);  /* 主循环必须周期性调用 */
uint8_t bsp_key_is_down(void);          /* 消抖后的稳定电平 */
uint8_t bsp_key_take_press(void);       /* 取"按下"事件，读后清零 */
uint8_t bsp_key_take_release(void);     /* 取"松开"事件 */
uint32_t bsp_key_press_count(void);     /* 累计按下次数 */
```

---

## 3. 关键参数

| 项 | 值 | 依据 |
|---|---|---|
| LED 引脚 | **PF1** | 原理图 |
| LED 极性 | **低电平点亮**（3V3 → 470Ω → LED → PF1） | 原理图 |
| LED 电流 | (3.3−1.8)/470 ≈ **3.2 mA** | 计算 |
| 按键引脚 | **PA0** | 原理图 |
| 按键极性 | **按下为低**（外部 R24 10k 上拉 + 内部上拉） | 原理图 + 代码 |
| 消抖时间 | **20 ms** | `KEY_DEBOUNCE_MS` |
| 硬件消抖 | C18 100nF 并联在按键两端 | 原理图 |

---

## 4. 时序/数据流

```
app_loop()（>10kHz 调用）
    │
    ├─ bsp_key_poll(now)
    │     ├─ 读 PA0 原始电平
    │     ├─ 与上次原始电平比较 → 变了就记时间戳，直接返回
    │     └─ 原始电平稳定 ≥20ms → 更新稳定电平 → 产生边沿事件
    │
    └─ key_action()
          └─ bsp_key_take_press() → bsp_led_toggle() + 打印日志
```

**为什么不用 EXTI 外部中断**：
- 按键是唯一一个用户输入，20ms 消抖本身就需要定时轮询；
- 用 EXTI 还得再配一个定时器做消抖，多一套中断源；
- 主循环本来就 >10kHz 轮询，采样率远超需求，且不会和电流环抢中断。

**消抖逻辑要点**：只有"原始电平稳定超过 20ms"才更新稳定电平并产生事件。
松手抖动不会产生多次翻转（因为每次抖动都会重置时间戳）。

---

## 5. 依赖

| 依赖谁 | 用途 |
|---|---|
| `board_cfg.h` | `LED_PIN_HW` / `KEY_PIN_HW` / 极性 / 消抖时间 |
| HAL GPIO | `HAL_GPIO_ReadPin` / `HAL_GPIO_WritePin` |
| `HAL_GetTick()` | 消抖计时（1ms 分辨率） |

被依赖：`app_main.c` 的 `key_action()`。

---

## 6. 验证方法（S1 台架清单）

1. **上电状态**：复位后 LED 应为**灭**（`bsp_gpio_init` 会把 PF1 置高）。
   > 注：`MX_GPIO_Init` 先把 PF1 置低，所以上电会有几毫秒的闪亮，属正常，可当"上电指示"。
2. **按一下亮、再按一下灭**：连续按 10 次，LED 状态应翻转 10 次，串口打印 `[KEY] pressed #N`。
3. **抖动测试**：快速轻点（不按到底），不应产生多次翻转。
4. **长按**：按住不放，只应触发 1 次（事件只在边沿产生）。
5. **串口对拍**：`led on` / `led off` / `led toggle` 命令应和按键效果一致。

---

## 7. 已知限制与 TBD

- 【已知限制】按键无长按/双击识别，需要时在 `key_action()` 里扩展（记录按下持续时间即可）。
- 【已知限制】`bsp_key_poll()` 依赖主循环频率；如果将来主循环被长任务阻塞 >20ms，消抖会受影响。
  届时把 `bsp_key_poll()` 挪到 1ms 定时任务里。
- 【TBD】如果板载 LED 亮度不合适（3.2mA 偏暗），可把 R37 改小到 220Ω。
