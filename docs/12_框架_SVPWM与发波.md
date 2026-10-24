# 子框架 · SVPWM 与发波（TIM1）

---

## 1. 职责边界

- **管**：TIM1 的计数模式/ARR/CCR 映射、三相互补输出、死区、MOE 使能与急停封波、
  SVPWM 算法（αβ → 三相占空比）。
- **不管**：电流环 PI（在 `16_框架_FOC控制算法.md`）、过流判断（在 `17_框架_保护与故障.md`）。

---

## 2. 接口

```c
/* User/Bsp/bsp_pwm.h */
void     bsp_pwm_init(void);                      /* 起波，占空比 0（三路 50% 且 MOE 关） */
void     bsp_pwm_enable(uint8_t en);              /* MOE 使能/封波 */
void     bsp_pwm_set_duty(float da, float db, float dc);  /* 0.0~1.0，含饱和 */
void     bsp_pwm_set_ccr(uint16_t c1, uint16_t c2, uint16_t c3); /* 直写，最快路径 */
uint16_t bsp_pwm_get_arr(void);
void     bsp_pwm_test_fixed(float duty);          /* S2 自检：三相固定占空比 */

/* User/Alg/alg_svpwm.h */
typedef struct { float ta, tb, tc; } svpwm_out_t;   /* 三相占空比 0~1 */
void alg_svpwm_calc(float valpha, float vbeta, float vdc, svpwm_out_t *out);
```

---

## 3. 关键参数（当前 TIM1 配置）

| 项 | 值 | 来源 |
|---|---|---|
| 计数模式 | 中心对齐 1（`TIM_COUNTERMODE_CENTERALIGNED1`） | tim.c |
| PSC | 0 | tim.c |
| ARR | **4249** | tim.c |
| PWM 频率 | 170MHz / (2 × 4250) = **20.000 kHz** | 计算 |
| 占空比分辨率 | ARR+1 = **4250** 级（≈11.05 bit） | — |
| RCR | 0（每半周期一个 UEV） | tim.c |
| CH1/1N | PA8 / PA7 | tim.c |
| CH2/2N | PA9 / PB0 | tim.c |
| CH3/3N | PA10 / **PF0** | tim.c |
| CH4 | 无引脚输出，CCR4=4249，**仅作 ADC 触发** | tim.c |
| 死区 DTG | **4 → 23.5 ns 标称（实际生效 60ns，由 EG3033 内部兜底）** | tim.c 现值 10，S2 改 4 |
| OSSR / OSSI | 均 DISABLE | tim.c |
| Break | DISABLE（无 BKIN 引脚） | tim.c |
| AutomaticOutput | DISABLE（MOE 需软件置位） | tim.c |

### 3.1 PWM 频率的精确口径

`ARR=4249`，中心对齐计数为 `0→ARR→0`，一个周期 2×(ARR+1)=8500 个计数：
`f = 170e6 / 8500 = 20000.0 Hz`（CubeMX 口径）。
若严格按 RM0440 "0→ARR−1→ARR→1" 的描述则为 2×ARR=8498 → 20004.7Hz，
差异 **0.023%**，对 FOC 无影响。**S2 用示波器实测记录真实值**（要求实测偏差 <0.1%）。

### 3.2 CCR ↔ 占空比映射

中心对齐 PWM1 模式下：`duty = CCR / (ARR+1)`，所以

```c
CCR = (uint16_t)(duty * 4250.0f + 0.5f);
CCR = clamp(CCR, CCR_MIN, CCR_MAX);
```

**为什么必须 clamp**：
- `CCR = 0` → 上管全关、下管全开，低侧采样窗口最长但桥臂无法输出；
- `CCR = ARR` → 下管全关，本周期无法采电流（INA240 在下管导通时才有电流）；
- 建议 `CCR_MIN = 40`（≈0.94%，约 470ns 最小脉宽，匹配死区+驱动器传输延迟），
  `CCR_MAX = 4210`。

### 3.3 死区：EG3033 + AO4606 联合分析（已定 DTG=4）

#### 3.3.1 硬件事实

| 项 | 参数 | 来源 |
|---|---|---|
| 驱动器 | EG3033（三相 P/N MOS 驱动） | 手册 V1.2 |
| **内部死区** | **DT = 60 ns（typ，无负载电容）** | 手册 §7.2 |
| 内部互锁 | `HIN=LIN=1` 或 `HIN=LIN=0` → **两管全关** | 手册真值表 |
| HO（P-MOS）灌电流 | **仅 40 mA**（关断慢的主因） | 手册 §7.2 |
| HO 上升时间 Tr | 80 ns（= P-MOS 关断） | 手册 §7.2 |
| LO（N-MOS）开通 | Ton 90 ns + Tr 280 ns | 手册 §7.2 |
| LO 关断 | Toff 30 ns + Tf 60 ns | 手册 §7.2 |
| MOSFET | AO4606：Ciss=255pF，Qg=5.2nC，Vgs(th)=1.2~2.4V | 手册 |

#### 3.3.2 关键结论 1：TIM1 极性与 EG3033 天然兼容

TIM1 互补输出在死区期间 **CHx=CHxN=0** → `HIN=LIN=0` → EG3033 输出**双关**。
MOE=0 封波时同理。所以**不需要任何极性取反**，也不存在"死区反而让两管直通"的风险。

#### 3.3.3 关键结论 2：AO4606 有约 96ns 的"自然死区"

真正的危险窗口是"**上管还没完全关断，下管就开了**"。逐段算：

| 时刻 | 事件 | 耗时 |
|---|---|---|
| T0 | HIN 变低，HO 开始把 P-MOS 栅极往 VCC2 拉 | — |
| T0+~61ns | P-MOS 的 Vgs 从 −11V 升到 −2.4V（Vgs(th)），**开始关断** | 80ns × (1−2.4/11) |
| T0+~80ns | HO 完全到 VCC2，**彻底关断** | Tr=80ns |
| T_dead 结束 | LIN 变高，LO 开始爬升 | 取决于 TIM1 死区 |
| +90ns | LO 传输延迟结束，栅压开始上升 | Ton |
| +157ns | N-MOS 的 Vgs 到 2.4V，**开始导通** | 90 + 280×(2.4/11) |

即：**P 管在 ~80ns 就关断了，N 管要到死区结束后 157ns 才开始导通**
→ 只要 `死区 + 157ns > 80ns`（永远成立），就不会直通。
**即使 TIM1 死区设成 0 也安全**，天然裕量约 96ns。

#### 3.3.4 实际取的值

```c
/* board_cfg.h */
#define PWM_DTG_REG   4u      /* 4 × 5.882ns = 23.5ns */
```

**实际生效死区 = max(23.5ns, EG3033 内部 60ns) = 60ns**。
设 23.5ns 或设 0，效果完全一样 —— 这是驱动内部 DT 兜底的结果，不是配置错误。

**为什么不再加大**：死区每多 1µs，占空比损失 2%，低速小电流时相电流畸变明显。
60ns 相对于 50µs 周期只有 0.12%，几乎无畸变，是这块硬件的最优解。

#### 3.3.5 DTG 编码速查表（t_DTS = 1/170MHz = 5.882ns，CKD=DIV1）

| DTG[7:5] | 死区公式 | 范围 |
|---|---|---|
| `0xx` | DT = DTG[7:0] × t_DTS | 0 ~ 747 ns |
| `10x` | DT = (64 + DTG[5:0]) × 2 × t_DTS | 753 ns ~ 1.50 µs |
| `110` | DT = (32 + DTG[4:0]) × 8 × t_DTS | 1.51 ~ 3.01 µs |
| `111` | DT = (32 + DTG[4:0]) × 16 × t_DTS | 3.02 ~ 6.02 µs |

| 目标 | DTG 寄存器值 | 实际 |
|---|---|---|
| **23.5 ns（本项目）** | **4** | 23.5 ns（实际 60ns） |
| 58.8 ns（原 CubeMX 值） | 10 | 58.8 ns（实际 60ns，无区别） |
| 300 ns | 51 (0x33) | 300 ns（会真的变成 300ns） |
| 500 ns | 85 (0x55) | 500 ns |
| 1 µs | 149 (0x95) | 999.9 ns |

> 分界点在 **60ns**：DTG ≤ 10 时被驱动内部 DT 吃掉；DTG ≥ 11 才是真正生效的死区。

### 3.4 SVPWM 算法（零序注入法，推荐实现）

相比"扇区判断 + 作用时间"版本，零序注入版本代码量小一半、无扇区边界 bug，效果等价：

```
1) Clarke 得到 vα, vβ
2) 三相正弦：
   va = vα
   vb = -0.5·vα + 0.8660254·vβ      (√3/2)
   vc = -0.5·vα - 0.8660254·vβ
3) 归一化到 Vdc：
   va /= vdc, vb /= vdc, vc /= vdc   → 范围 ±0.577 以内为线性区
4) 零序注入（居中）：
   vmax = max(va,vb,vc); vmin = min(va,vb,vc)
   voff = -(vmax + vmin) * 0.5f
5) 占空比：
   ta = va + voff + 0.5f     ← 加 0.5 抬到 0~1
   tb = vb + voff + 0.5f
   tc = vc + voff + 0.5f
```

**线性区上限**：`|V| ≤ Vdc/√3`（相电压幅值），SVPWM 比正弦调制多 15.47% 利用率。
超过后进入过调制，必须先做**圆限幅**：

```
vmax_lin = vdc * 0.57735027f     /* 1/√3 */
if (vα²+vβ² > vmax_lin²) 按比例缩回（保持角度不变）
```

**为什么用居中注入而不是七段式查表**：中心对齐 PWM + 零序注入 = 七段式，
数学上完全等价，但前者能直接复用 TIM1 中心对齐的硬件对称性，不用管扇区切换。

---

## 4. 时序/数据流

```
[ADC JEOC ISR]
   foc_core 算出 vα,vβ (V)
        │
        ▼
   alg_svpwm_calc(vα, vβ, vdc, &out)   ← vdc 来自母线采样（TBD）或固定标称值
        │
        ▼
   bsp_pwm_set_duty(out.ta, out.tb, out.tc)
        │
        ▼
   TIM1->CCR1/2/3 (影子寄存器，下个 UEV 生效)
```

**CCR 预装载必须开启**（`AutoReloadPreload=ENABLE` 已配，OC 通道默认 OCPreload 使能），
保证 CCR 在 UEV 时整体更新，避免三路不同步导致的瞬时错相。

---

## 5. 依赖

| 依赖谁 | 用途 |
|---|---|
| 时钟 | TIM1 时钟 170MHz |
| `cfg_foc.h` | PWM 频率、CCR_MIN/MAX、死区值 |
| （可选）ADC Vbus | SVPWM 归一化 |

被依赖：`alg_foc`、`bsp_adc`（CC4 触发源）、`17_框架_保护与故障.md`（封波）。

---

## 6. 验证方法（S2 台架清单）

1. **频率**：示波器测任一路 PWM，应为 20.000kHz ±0.1%。
2. **互补与死区**：同时抓 CH1(P A8) 与 CH1N(PA7)，测上升沿到上升沿的间隔 = 死区设定值。
   六路全测一遍（三相死区应一致）。
3. **占空比线性**：`bsp_pwm_test_fixed(0.1 / 0.5 / 0.9)` 三档，实测占空比误差 <1%。
4. **相序**：固定 `ta=0.7, tb=0.4, tc=0.4`，用示波器确认 CH1 高电平最宽 → 相序正确。
5. **封波**：`bsp_pwm_enable(0)` 后六路必须全部为低（或按 OSSI 设定进入安全态），
   且下管不能常开（否则电机刹车）。**【TBD】OSSR/OSSI 是否要改成 ENABLE？**
   建议：OSSI=ENABLE（空闲时输出空闲电平=低），OSSR=DISABLE（运行中不强制）。

---

## 7. 已知限制与 TBD

- 【已定】死区 DTG=4（23.5ns 标称，实际 60ns），计算见 3.3 节
- 【TBD】CCR_MIN 是否要留更大余量（取决于电流采样最小脉宽需求）
- 【TBD】Vbus 用实测值还是固定标称值（影响 SVPWM 归一化精度）
- 【已知限制】ARR 只有 4250 级，若将来提高 PWM 频率到 40kHz，分辨率降到 2125 级（≈11 bit），
  高调制比时会出现占空比量化抖动，需评估。
- 【已知限制】无 BKIN 引脚，硬件过流封波不可用，见 `17_框架_保护与故障.md`。
