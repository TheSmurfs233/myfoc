# 子框架 · FOC 控制算法

---

## 1. 职责边界

- **管**：Clarke/Park/反 Park 变换、dq 轴 PI 电流调节器、解耦前馈、电压限幅、电流环离散化与整定方法。
- **不管**：SVPWM 占空比计算（`12_框架_SVPWM与发波.md`）、角度来源（`14`）、保护（`17`）。

---

## 2. 接口

```c
/* User/Alg/alg_foc.h */
typedef struct {
    /* 输入 */
    float ia, ib, ic;        /* A */
    float elec_rad;          /* 电角度 rad */
    float vdc;               /* 母线电压 V */
    float id_ref, iq_ref;    /* A */
    /* 输出 */
    float id, iq;            /* A 实测 */
    float vd, vq;            /* V 指令 */
    float valpha, vbeta;     /* V */
    float ta, tb, tc;        /* 占空比 0~1 */
    /* 内部状态 */
    float i_alpha, i_beta;
    float integ_d, integ_q;
} foc_t;

void  foc_init(foc_t *f);
void  foc_step(foc_t *f);     /* 全部计算，20kHz 调用 */
void  foc_reset(foc_t *f);    /* 清积分器（故障/切换状态时） */

/* User/Alg/alg_pi.h */
typedef struct {
    float kp, ki;
    float out_min, out_max;
    float integ;
    float out;
} pi_t;
float pi_calc(pi_t *p, float err, float ts);
void  pi_reset(pi_t *p);
```

---

## 3. 关键参数

### 3.1 变换公式（等幅值 Clarke，非等功率）

```c
i_alpha = ia;
i_beta  = (ia + 2.0f * ib) * INV_SQRT3;      /* INV_SQRT3 = 0.57735027 */

/* 也可用 ic 提高鲁棒性：i_beta = (ib - ic) * INV_SQRT3 */
```

**为什么用等幅值**：变换后电流幅值 = 相电流峰值，物理意义直观，PI 参数整定简单。
代价是功率不守恒（差 2/3），但不影响控制。

Park（用查表或 CORDIC 得到的 sin/cos）：

```c
id =  i_alpha * cos_t + i_beta * sin_t;
iq = -i_alpha * sin_t + i_beta * cos_t;
```

反 Park：

```c
valpha = vd * cos_t - vq * sin_t;
vbeta  = vd * sin_t + vq * cos_t;
```

### 3.2 sin/cos 的获取

| 方案 | 精度 | 耗时@170MHz | 适用 |
|---|---|---|---|
| `sinf()/cosf()` | 高 | ~1.5µs（两个） | 太慢，不推荐 |
| **查表 1024 点 + 线性插值** | 0.0003° | ~0.3µs | **推荐** |
| G4 硬件 CORDIC | 高 | ~0.2µs（含握手） | 最优，但配置复杂 |
| 查表 4096 点不插值 | 0.044° | ~0.15µs | 够用，RAM 占 16KB（太大） |

**推荐**：1024 点 float 表（4KB）或 q15 表（2KB），主频 170MHz 下线性插值误差可忽略。
S6 先用 1024 点表，若 ISR 时间富余再换 CORDIC。

### 3.3 PI 调节器（离散化 + 抗饱和）

```c
float pi_calc(pi_t *p, float err, float ts)
{
    float out = p->kp * err + p->integ;
    /* 抗饱和：输出限幅后，用饱和差值回算积分（back-calculation） */
    if (out > p->out_max) {
        out = p->out_max;
        p->integ += p->ki * err * ts;          /* 可选：只在未饱和时积分 */
        p->integ = clampf(p->integ, p->out_min, p->out_max);
    } else if (out < p->out_min) {
        out = p->out_min;
        p->integ += p->ki * err * ts;
        p->integ = clampf(p->integ, p->out_min, p->out_max);
    } else {
        p->integ += p->ki * err * ts;
    }
    p->out = out;
    return out;
}
```

**离散化说明**：这里积分项已经吸收了 `Ts`，所以 `ki` 的单位是 `V/(A·s)`，
与连续域 PI 的 `Ki` 一致，便于按电机参数直接算。

### 3.4 PI 参数整定公式（按电机电气参数）

电机 dq 轴电压方程（忽略交叉耦合）：

```
vd = R·id + L·did/dt
vq = R·iq + L·diq/dt
```

按"零极点对消 + 目标带宽 ωc"整定：

```
Kp = ωc · L
Ki = ωc · R
```

| 参数 | 符号 | 典型值 | 说明 |
|---|---|---|---|
| 目标电流环带宽 | fc | **500 Hz ~ 1 kHz** | 20kHz 采样下 fc ≤ fs/10 才稳 |
| ωc | 2π·fc | 3140 ~ 6280 rad/s | |
| 相电阻 | R | **2.3 Ω**（线间 4.6/2） | 用户实测 |
| 相电感 | L | **0.5 mH**（取平均，够电流环整定） | 估计 + 电流环实测验证 |

例：R=0.5Ω，L=1mH，fc=1kHz → Kp = 6280×0.001 = **6.28**，Ki = 6280×0.5 = **3140**。

**电压限幅**：`out_max = vdc × 0.577`（线性区），`out_min = -out_max`。
vd/vq 合成后再做圆限幅：

```c
float vmag = sqrtf(vd*vd + vq*vq);
float vlim = vdc * 0.57735027f;
if (vmag > vlim) { float k = vlim / vmag; vd *= k; vq *= k; }
```

### 3.5 解耦前馈（S6 后期加，先不加也能转）

```
vd_ff = -ωe · L · iq
vq_ff =  ωe · L · id + ωe · ψf
```

其中 ψf = 永磁磁链（可由反电动势常数 Ke 换算：ψf = Ke_LL/(√3·2π·f) ... 需实测）。
**先不加前馈**，用 PI 自己扛交叉耦合，转速上去后再加。

### 3.6 标幺化 vs SI 单位（决策）

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **SI 单位（float）** | 可读性最好、调参直观、文档和代码一致 | 多花一点周期（G431 有单精度 FPU，可接受） | ✅ **本项目选这个** |
| q 格式（定点） | 快、无 FPU 依赖 | 调参痛苦、易溢出、文档难写 | 不选 |

G431 是 Cortex-M4**F**，单精度硬浮点，20kHz 下浮点运算开销完全可接受。
**注意**：全程用 `float` 不用 `double`（double 是软件模拟，慢 10 倍以上）。

---

## 4. 时序/数据流

```
[ADC JEOC ISR @20kHz]
        │
        ├─ 读 ia/ib/ic（已换算）
        ├─ mid_angle_update() → elec_rad
        │
        ▼
   foc_step(&foc)
        │
        ├─ Clarke:  ia,ib → i_alpha,i_beta
        ├─ Park:    i_alpha,i_beta,θ → id,iq
        ├─ 误差:    ed = id_ref - id;  eq = iq_ref - iq
        ├─ PI:      vd = pi_calc(&pi_d, ed, TS);  vq = pi_calc(&pi_q, eq, TS)
        ├─ 限幅:    vd,vq 圆限幅到 vdc/√3
        ├─ 反 Park: vd,vq,θ → valpha,vbeta
        ├─ SVPWM:   valpha,vbeta,vdc → ta,tb,tc
        └─ 写 CCR1/2/3
```

**控制周期 TS = 50 µs**。

---

## 5. 依赖

| 依赖谁 | 用途 |
|---|---|
| `mid_current` | 三相电流（A） |
| `mid_angle` | 电角度 |
| `alg_svpwm` | 占空比 |
| `bsp_pwm` | CCR 写入 |
| `cfg_foc.h` | R、L、极对数、限幅、带宽参数 |

---

## 6. 验证方法（S6 台架清单）

1. **变换自检（不开电机）**：注入已知 `ia=1, ib=-0.5, ic=-0.5`，
   `ialpha=1, ibeta=0`；θ=0 时 `id=1, iq=0`；θ=90° 时 `id=0, iq=-1`（或 +1，看符号定义）。
   **先在 PC/Excel 对拍，再上台架。**
2. **开环电流注入**：`IDLE` 状态强制 `vd=常数, vq=0, θ=0`，
   测实际 id 应跟随（这是电流环开环验证）。
3. **闭环阶跃**：`iq_ref` 从 0 阶跃到 1A，示波器抓 iq，上升时间 ≈ 1/(2·fc)，超调 <10%。
4. **Id 保持**：稳态时 id 应 ≈ 0（±0.05A）。
5. **带宽**：`iq_ref` 给 500Hz 小正弦（0.2A），输出幅值衰减应 < 3dB。
6. **ISR 时间**：GPIO 在 ISR 进出翻转，示波器测 < 8µs（目标 <16% 占用）。

## S8 追加：测速与速度环的两处关键结构

| 结构 | 位置 | 作用 |
|---|---|---|
| **PLL 速度观测器** | `alg_pll.c` / `mid_enc.c` | 二阶 PLL（14 Hz、ζ0.707）跟踪电角度，替代 10 ms 窗口平均；测速延迟约 15 ms -> 11 ms，800 rpm 纹波 0.15 A -> 0.06 A |
| **加速度前馈** | `alg_spd.c` | `u += (J/Kt)·d(ref)/dt`，`J/Kt = 3.1e-3`；缺它时 600 rpm/s 斜坡所需的 0.2 A 全靠积分建立，100 rpm 阶跃超调 96%，加上后 8% |

| 速度环实测 | 值 |
|---|---|
| 执行频率 | 1 kHz（每 20 个电流帧） |
| 闭环带宽 | 约 3 Hz（延迟预算 26 ms：PLL 16 ms + 反馈低通 10 ms） |
| 相位裕度 | 约 26 度（因此 kp 只能到 0.010） |
| 阶跃响应 | 超调 2~5%，稳态误差 <=0.9%（受 600 rpm/s 斜坡限制） |

---

## 7. 已知限制与 TBD

### 已闭环（S4~S6 实测确认）

| 项 | 结论 |
|---|---|
| 相电阻 R / 相电感 L | R≈2.3 Ω（线间 4.6/2）、L 取 0.5 mH，电流环 PI 按此整定 |
| 电机极对数 | 7（已用于电角度换算） |
| 电流环带宽 | 实测阶跃 rise 0.2 ms、超调 <7%、纹波 10~14 mA |
| 角度补偿 | 不需要：20 kHz 下 1.5Ts 补偿会引入额外噪声，实测高速电流波形良好 |

### 仍然存在的限制（不影响使用）

- 【已知限制】未做死区补偿。实测电流纹波仅 10~14 mA，死区影响可忽略，故不实现。
- 【已知限制】未做 dq 轴参数不匹配补偿（Ld≠Lq），用平均 L 整定即可。
- 【已知限制】未做 MTPA / 弱磁（12 V + 7 极对的场景不需要）。
- 【已知限制】电流环计算延迟 3~5 µs，但速度环只有 3 Hz，可忽略。
