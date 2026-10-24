# 子框架 · ADC 三相电流采样

---

## 1. 职责边界

- **管**：三相电流的硬件链路、ADC 触发时序、原始码值 → 安培的换算、零点校准、采样质量诊断。
- **不管**：电流环 PI、过流保护动作（分别见 16、17 号文档）。

---

## 2. 接口

```c
/* User/Mid/mid_current.h */
typedef struct {
    float ia, ib, ic;       /* 安培，已扣零漂、已换增益 */
    uint16_t raw[3];        /* 原始码值，调试用 */
    float offset[3];        /* 零漂码值（上电校准时确定） */
    uint8_t valid;
} current_sample_t;

void  mid_current_init(void);            /* 零漂校准：PWM 关断时采 N 次取均值 */
void  mid_current_update(uint16_t j1, uint16_t j2, uint16_t j3);  /* ISR 内调用 */
const current_sample_t *mid_current_get(void);

/* User/Bsp/bsp_adc.h */
void bsp_adc_init(void);                 /* 校准 + 使能注入组 + 使能中断 */
void bsp_adc_start(void);                /* 启动注入组，等待 TIM1_CC4 触发 */
```

---

## 3. 关键参数

### 3.1 硬件链路

```
U 相下管源极 ── R1 25mΩ ── PGND
                    │  IN+(IU+)          INA240A2 (U6)  增益 50V/V
                    │  IN-(IU-)          REF1=AGND, REF2=A3V3 → VREF=1.65V
                    │                    OUT ─ R18 10Ω ─ ISU ─→ PA4 (ADC2_IN17, JDR1)
                    │
V 相 ── R2 25mΩ ── 同理 ─ INA240 (U2) ─ ISV ─→ PA5 (ADC2_IN13, JDR2)
W 相 ── R3 25mΩ ── 同理 ─ INA240 (U3) ─ ISW ─→ PA6 (ADC2_IN3,  JDR3)
```

| 项 | 值 | 备注 |
|---|---|---|
| ADC | ADC2，12bit，单端 | adc.c |
| ADC 时钟 | SYSCLK/4 = 42.5MHz | adc.c |
| 采样时间 | 24.5 cycles（注入组） | adc.c |
| 单通道转换 | (24.5 + 12.5)/42.5MHz = **0.871 µs** | 计算 |
| 3 通道总耗时 | **2.61 µs** | 计算 |
| 触发源 | `TIM1_CC4` 上升沿（`ADC_EXTERNALTRIGINJEC_T1_CC4`） | adc.c |
| 中断 | `ADC1_2_IRQn`（JEOC），优先级需改 0 | it.c |
| INA240 增益 | **50 V/V**（A2 型号） | 硬件 |
| 参考电压 | 3.3V（VDDA=VREF+，无外部基准） | 原理图 |
| 采样电阻 | **25 mΩ**（实际焊接，原理图标 10mΩ） | 实测/用户确认 |
| INA240 偏置 | REF1=AGND, REF2=A3V3 → **VREF=1.65V** | 原理图核对 |

### 3.2 采样时刻为什么选在三角波峰顶

中心对齐模式下，`CNT < CCR` 时上管开、下管关；`CNT > CCR` 时上管关、下管开。
三角波**峰顶**（CNT=ARR）时，只要占空比 <100%，`CNT > CCRx` 必然成立 →
**三相下管同时导通** → 三个低侧 shun 上都有对应相电流 → 可同时刻采样。

峰顶同时也是**下管导通窗口的中心**，距离两侧开关沿最远 → 开关噪声最小。
这是低侧三电阻方案的标准采样点。

CC4 怎么产生上升沿：`CH4` 配 PWM1 模式、`CCR4 = ARR = 4249`。
PWM1 下 `CNT < CCR4` 输出高，只有峰顶那 1~2 个计数输出低 →
每个周期在峰顶之后立刻产生一个上升沿，即每周期触发一次注入组。✓

### 3.3 码值 → 安培换算

```
V_adc   = raw × 3.3V / 4096                     (0.806 mV/LSB)
V_sense = (V_adc − V_ref) / 50                  (INA240A2 增益 50)
I_phase = V_sense / R_shunt
```

合并：

```
I = (raw − raw_offset) × (3.3 / 4096) / 50 / R_shunt
  = (raw − raw_offset) × K
```

| Rshunt | K（A/LSB） | 双向量程 | 说明 |
|---|---|---|---|
| 10 mΩ | 1.61 mA | ±3.3 A | 原理图标注值（实物不是这个） |
| **25 mΩ（本项目）** | **0.645 mA** | **±1.32 A** | 实际焊接值，代码按此配置 |

灵敏度 = Rshunt × 50 = **1.25 V/A**；
量程 = 1.65V（VREF）÷ 1.25V/A = **±1.32 A**；
分辨率 = 0.806mV ÷ 1.25V/A = **0.645 mA/LSB**。

⚠ **±1.32A 很小**。若电机额定电流 >1A，S3 标定时会看到 ADC 贴边（码值 0 或 4095，切顶），
届时只能换更小阻值的采样电阻（10mΩ → ±3.3A，5mΩ → ±6.6A）。

**已确认（2026-10-25 核对原理图）**：INA240 的 `REF1 = AGND`、`REF2 = A3V3`，
所以 `VREF = (0 + 3.3)/2 = 1.65V`，是标准的**双向**偏置：

- 零电流 → 1.65V → 码值 **2048**
- 正电流 → 码值上升，负电流 → 码值下降，**正负都能测**
- 满量程 ±1.32A（受 1.65V / 1.25V·A⁻¹ 限制）

代码里对应 `board_cfg.h`：

```c
#define RSHUNT_OHM    0.025f
#define INA240_GAIN   50.0f
#define CUR_A_PER_LSB ((ADC_VREF / ADC_FULL_SCALE) / (INA240_GAIN * RSHUNT_OHM))  /* 0.645mA */
#define CUR_ZERO_CODE (ADC_FULL_SCALE * 0.5f)                                     /* 2048 */
#define CUR_RANGE_A   ((ADC_VREF * 0.5f) / (INA240_GAIN * RSHUNT_OHM))            /* ±1.32A */
```

---

## 4. 时序/数据流

```
TIM1 CNT 峰顶
    │
    ├─ CC4 上升沿 ──▶ ADC2 注入组启动（硬件，无需 CPU）
    │                     │
    │                     ├ rank1: PA4 → JDR1   0.871us
    │                     ├ rank2: PA5 → JDR2   0.871us
    │                     └ rank3: PA6 → JDR3   0.871us
    │                                    │
    │                          JEOC ─────┘ (t = 2.61us)
    │                                    ▼
    │                          ADC1_2_IRQHandler
    │                                    ▼
    │                          HAL_ADCEx_InjectedConvCpltCallback(&hadc2)
    │                                    ▼
    │                          mid_current_update(JDR1, JDR2, JDR3)
    │                                    ▼
    └──────────────────────────  foc_core_step()
```

**读 JDR 必须直读寄存器**，不要用 `HAL_ADCEx_InjectedGetValue()`（函数调用+多次判断，多花几百 ns）：

```c
uint16_t j1 = (uint16_t)hadc2.Instance->JDR1;
uint16_t j2 = (uint16_t)hadc2.Instance->JDR2;
uint16_t j3 = (uint16_t)hadc2.Instance->JDR3;
```

---

## 5. 依赖

| 依赖谁 | 用途 |
|---|---|
| TIM1 CH4 | 触发源，必须在 ADC 使能前配置好 |
| 时钟 | ADC 42.5MHz |
| `cfg_foc.h` | RSHUNT、INA_GAIN、校准次数 |

被依赖：`alg_foc`、`17_框架_保护与故障.md`（过流判断）。

---

## 6. 验证方法（S3 台架清单）

1. **零漂**：PWM 全关、电机不接电，读 raw[0..2]，三相应都在 2048±20 内。偏差大 → INA240 REF 接法问题。
2. **极性**：给某相注入已知小直流（如串一个 1A 恒流源或有刷电源），
   raw 应朝正方向变化，且三相串扰 <1%。
3. **噪声**：PWM 起波但占空比 0（电机不转），读 raw 的峰峰值。
   目标：抖动 < ±8 LSB（≈±2.6% 满量程）；若过大 → 采样时刻偏了或需要 RC 滤波。
4. **跟随性**：串口上报 raw 波形，手动转动电机，三相波形应为相位差 120° 的正弦。
5. **线性度**：用 2 个已知电流点核对 K 值（如 1A 与 5A），误差 <3%。

---

## 7. 已知限制与 TBD

### 7.1 单 ADC 顺序采样的相位误差（重要）

三相互差最大 **2.61 µs**。以电频率 f_e 计算，误差角 = `2π·f_e·2.61µs`：

| 电频率 | 电角误差（rank1 vs rank3） | 影响 |
|---|---|---|
| 100 Hz | 0.094° | 可忽略 |
| 500 Hz | 0.47° | 可忽略 |
| 1000 Hz | 0.94° | 可忽略 |
| 2000 Hz | 1.88° | 增益误差 ~0.05%，可接受 |

**结论：20kHz PWM + 常见电机（电频率 < 1kHz）下，单 ADC 顺序采样完全够用，不必上双 ADC。**
（真实误差还包含采样保持的孔径时间，但量级相同。）

**唯一例外**：若三相电流严重不平衡或转速极高（>30000rpm+多极对），
再考虑 ADC1+ADC2 双 ADC 同步（ADC1 采 1 相、ADC2 采 2 相，耗时降到 1.74µs）。

### 7.2 其他

- 【已定】Rshunt=25mΩ、INA240 REF1=AGND/REF2=A3V3（VREF=1.65V）、VREF+=3.3V
- 【已知】INA240 输出已有 R18/R6/R8 10Ω + 22pF（截止≈723MHz，仅防振，无抗混叠作用）；若噪声大可改 1kΩ+1nF
- 【已知限制】ADC 常规组里多配了一个 CH17（`adc.c` 第 69~78 行），
  与注入组 rank1 重复，因软件触发且未使能所以不会执行。
  **建议**：CubeMX 里把常规组关掉（NbrOfConversion=0），或改作 Vbus 采样通道。
- 【已知限制】`Overrun = ADC_OVR_DATA_PRESERVED`，注入组在 JEOC 未及时清除时会覆盖数据，
  不会丢触发，符合 FOC 需求（宁可丢一拍数据也不能停）。
- 【已知限制】零漂会随温度漂移（INA240 的 offset drift 约 1µV/°C → 0.5 LSB/°C@5mΩ），
  长时间运行需考虑跑中点自动校准（暂不做，S8 再评估）。
