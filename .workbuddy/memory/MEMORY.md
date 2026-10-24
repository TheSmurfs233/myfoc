# wb_foc 项目长期记忆

## 工程定性
- STM32G431KBT6（LQFP32，M4F 170MHz）+ MT6701 磁编（SPI1）+ 3×INA240A2（50V/V）三电阻低侧采样
- 控制频率 20kHz（TIM1 中心对齐，ARR=4249，CCR 分辨率 4250），ADC2 注入组由 TIM1_CC4 峰顶触发
- 工具链：CubeMX 6.18.1 + FW_G4 V1.6.3 + MDK-ARM V5.32

## 用户工作方式（硬要求）
- 一步一步来，听指令再动手；任何不确定先问，不自行假设
- 先给方案，用户确认后才执行
- 每次开发必须同步更新 docs/ 文档（框架/日志/进度三件套）

## 文档体系（docs/）
- README.md 索引；00 开发日志（倒序，只增不改）；01 项目进度（S0~S8 看板）
- 10 主框架；11 时钟；12 SVPWM发波；13 ADC电流采样；14 编码器；15 串口调试；16 FOC算法；17 保护故障；18 TBD 清单
- 每个框架文档固定 7 节：职责/接口/参数/时序/依赖/验证/TBD

## 关键技术决策（已定）
- 算法层不依赖 HAL（App→Alg→Mid→Bsp→HAL 单向）
- 采用 SI 单位 float（G431 有单精度 FPU），不用 q 格式
- SVPWM 用零序注入法（等价七段式），不做扇区判断
- 电流环只用 ADC JEOC 中断，禁止开 TIM1 更新中断（中心对齐 UEV 每半周期两次）
- 单 ADC 顺序采样可接受（电频率 1kHz 时误差 <1°），暂不上双 ADC

## 编译闭环（重要，每次改代码必做）
- 本机 Keil：`D:/installfile/Keil_v5`（ARMCC V5.06 update 5，**GBK 解析源码**）
- 命令行全量编译：
  `"D:/installfile/Keil_v5/UV4/UV4.exe" -b "<abs>/MDK-ARM/myfoc.uvprojx" -j0 -o "<abs>/MDK-ARM/build.log"`
  返回码 0=OK 1=警告 2=错误；看 build.log 的 `Error(s)` 行
- **源文件必须纯 ASCII**：ARMCC V5 按 GBK 解析，UTF-8 中文放字符串里会报
  `#8: missing closing quote`（中文注释多数能过但不能依赖）。中文只写 docs/
- 自检：`grep -rP "[\x80-\xff]" User/`
- **每次 build 换一个 log 文件名**（b1.log/b2.log/...）：Keil GUI 开着时会挡住同名 log 覆盖，
  读到旧 log 会误判成功；以返回码为准
- **Keil 是增量编译**：`__DATE__/__TIME__` 只在含它的 .c 被重编时才更新，
  判断"板子跑的是不是最新固件"**必须看 `FW_BUILD_ID`**，不能看时间戳
- 2026-10-25 基线：**Code=23378 RO=734 RW=68 ZI=8844**（RAM 8.9K/32K）

## 真机验收（串口，COM7 @1Mbaud）
- 一键：`python test/accept/s1_accept.py COM7 1000000`（**53 项断言**）
- 覆盖：help/ver/stat 内容与行数、FireWater 禁冒号、echo、错误分支、LED 回读、
  rate 钳位、状态机全部拒绝分支、波形 CSV 形状/帧率/通道/平滑度/丢帧
- 脚本内每个读操作带硬超时（quiet + max_seconds），板子 wave on 也不会挂死
- 实测基线：主循环 460~467kHz；波形 502 帧/s；drop=0；ch4 步进 ≤12.5

## 固件版本（硬规则）
- `User/Cfg/board_cfg.h` 的 **`FW_BUILD_ID`（当前 13）是唯一权威版本标识**，`ver` 打印 `... #13`
- **每次改固件必须 +1**；改这个头文件会触发全量重编，顺带刷新 `__TIME__`
- 验收脚本自动 grep 源码值 vs 板子上报值，不一致直接报 RE-FLASH

## ⚠ 烧录别下错工程（两个都叫 myfoc）
- 正确：`D:/Embedded/STM32/wb_foc/MDK-ARM/myfoc/myfoc.hex`（约 68 KB）
- 错误：`D:/Embedded/STM32/workspace/myfoc/MDK-ARM/...`（约 344 KB，旧工程）
- 下错表现：**所有命令都回 `ERR command`**（来自 workspace 的 App/debug_commands.c，本工程没有）
- 判据：`ver` 打印 `board = Board_v2/STM32G431KBT6` 且带 `#N`

## 主机仿真测试（重要，纯逻辑改动必跑）
- **WSL 不可用**：`wsl.exe` 被安全策略拉黑（Program Blacklist），无法绕过
- **宿主 gcc**：`D:/Xilinx/Vitis_HLS/2022.2/tps/win64/msys64/mingw64/bin/gcc.exe`（gcc 6.2.0）
- 一键：`bash test/host/build.sh`（编译 + 运行，返回码 0=全绿）
- 结构：`test/host/{test_main.c, build.sh, fake/{stm32g4xx_hal.h, main.h, usart.h, fake_api.h, hal_stub.c}}`
- 做法：`-I test/host/fake` 影子头文件覆盖真实 HAL，**生产代码零侵入、无 #ifdef HOST_TEST**
- 观测靠"可注入输入 + 可捕获输出"：按键电平 / TX 捕获 / RX 注入 / DMA 完成时序
- **fake RX 必须每写满一圈上报一次 `HAL_UART_RXEVENT_TC`**（Size=全尺寸），否则测不出整圈丢数据的 bug
- 用例之间要清 TX 环（pump+reset）、RX 环、`app_cmd_init()`，否则脏数据串台
- 当前 **158 断言全绿**；S4/S6 的算法数值验证 + 虚拟电机闭环走这里

## 调试口协议（硬规则，2026-10-25 已定为 FireWater）
- 单串口 USART1（1Mbaud，PB6/PB7→CN3），**波形 = FireWater 纯 CSV**
  帧：`ch0,ch1,...,ch5\n`（如 `169.347,1.000,0.000,2.000,-773.000,452.542`），3 位小数
  **冒号只能出现一次且在行首**（可选标签），绝不能每个通道带名字（否则解析失败→当文本显示→刷屏）
- 日志行统一加 **`[log] `** 前缀 + 禁止冒号 → VOFA+ 只能当文本，接收区不被波形刷屏
- **VOFA+ 必须选 FireWater**（选错格式会导致乱码 + 文本被吞）
- **铁律：`log_printf` 里禁止出现冒号 `:`**，一律用 `=`
  原因：FireWater 把每行按 `name:value` 解析，含冒号的行会被当数据吞掉、文本区看不见
  自检：`grep -rn 'log_printf("[^"]*:' User/` 输出必须为空
- 测试正弦 ch4 用 64 点表 + **线性插值**（不插值在 500Hz 下是阶梯）
- VOFA+ 的 `Δt` 需与速率匹配（500Hz → 2ms），否则曲线横向被压缩
- 速率：**默认=上限=500Hz**（`DBG_WAVE_RATE_HZ` 引用 `DBG_WAVE_RATE_MAX`，单点改）；CSV ~44B/帧占 22%；理论极限 1000Hz 不做
- 双串口方案（USART2→板载 USB）**已实现过又否决**，代码已删；若 S6 需要 1000Hz + 文本，可重新启用

## 未解决（阻塞项，S0.5 已闭环的已删除）
- **电机极对数** → S5 电角度换算需要，S4 开环可先不管
- **相电阻 R / 相电感 L** → S6 电流环 PI 整定需要
- 电机额定/堵转电流 → S7 保护阈值需要（可先按 1.32A 采样量程设一个保守值）
- Vbus 范围已定 12V；死区 DTG=4、Rshunt=25mΩ、MT6701 SSI 均已闭环
