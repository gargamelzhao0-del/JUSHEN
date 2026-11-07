# 四足机器狗控制器仿真测试指南

## 一、STM32代码修改指南（禁用超时错误处理）

### 1.1 需要修改的文件

在无电机测试时，需要禁用以下超时检测：

#### 修改1: `Core/Inc/motor_control.h` - 禁用超时检测

```c
// 找到约第115行
#define MOTOR_STATUS_TIMEOUT_MS  100

// 修改为 (将100改为0可禁用)
#define MOTOR_STATUS_TIMEOUT_MS  0  // 禁用超时检测
```

#### 修改2: `Core/Inc/motor_control_enhanced.h` - 禁用反馈超时

```c
// 找到约第27行
#define MOTOR_FEEDBACK_TIMEOUT_MS   100     /* 反馈超时时间（ms）*/

// 修改为
#define MOTOR_FEEDBACK_TIMEOUT_MS   0       // 禁用超时检测
```

#### 修改3: `Core/Inc/quad_command_fusion.h` - 禁用指令融合超时

```c
// 找到约第112行
#define FUSION_FEEDBACK_TIMEOUT_MS      100       /* 电机反馈超时（ms）*/

// 修改为
#define FUSION_FEEDBACK_TIMEOUT_MS      0         // 禁用超时检测
```

#### 修改4: `Core/Src/quad_motor_api.c` - 禁用超时错误标记

找到约第804行：
```c
// 原代码：
if (elapsed > MOTOR_STATUS_TIMEOUT_MS) {
    g_motor_status[motor_id].error_flags |= MOTOR_ERROR_FLAG_TIMEOUT;
}

// 修改为：
if (elapsed > MOTOR_STATUS_TIMEOUT_MS && MOTOR_STATUS_TIMEOUT_MS > 0) {
    g_motor_status[motor_id].error_flags |= MOTOR_ERROR_FLAG_TIMEOUT;
}
// 或者直接注释掉这两行
// if (elapsed > MOTOR_STATUS_TIMEOUT_MS) {
//     g_motor_status[motor_id].error_flags |= MOTOR_ERROR_FLAG_TIMEOUT;
// }
```

---

## 二、仿真测试脚本使用说明

### 2.1 纯位置模式仿真测试

```bash
# 完整分析（解析+仿真+波形图+3D动画+导出）
python quad_simulation_test.py --input "你的CANFD数据.csv" --full

# 仅解析和分析
python quad_simulation_test.py --input "你的CANFD数据.csv" --analyze

# 仅波形图
python quad_simulation_test.py --input "你的CANFD数据.csv" --waveforms

# 仅3D动画
python quad_simulation_test.py --input "你的CANFD数据.csv" --3d
```

**输出文件：**
- `simulation_output/waveforms_position.png` - 位置跟踪波形
- `simulation_output/waveforms_velocity.png` - 速度波形
- `simulation_output/waveforms_torque.png` - 扭矩波形
- `simulation_output/waveforms_error.png` - 跟踪误差波形
- `simulation_output/3d_gait.png` - 3D步态图
- `simulation_output/gait_topview.png` - 俯视图
- `simulation_output/simulation_summary.csv` - 汇总数据
- `simulation_output/simulation_detail.csv` - 详细数据

### 2.2 力位混合模式仿真测试

```bash
# 纯位置模式
python hybrid_simulation_test.py --input "你的CANFD数据.csv" --mode position

# 力位混合模式（模拟足端力）
python hybrid_simulation_test.py --input "你的CANFD数据.csv" --mode hybrid

# 对比两种模式
python hybrid_simulation_test.py --input "你的CANFD数据.csv" --mode compare
```

---

## 三、测试流程

### 3.1 完整测试流程

```
┌─────────────────────────────────────────────────────────────────┐
│                      测试流程                                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. [STM32] 修改超时检测参数（见上文）                          │
│       │                                                         │
│       ▼                                                         │
│  2. [STM32] 编译并烧录固件                                      │
│       │                                                         │
│       ▼                                                         │
│  3. [上位机] 运行 quad_test_simulator.py                        │
│       │  (模拟强化学习输出12关节角度)                            │
│       ▼                                                         │
│  4. [CANFD] 捕获CAN总线数据并导出CSV                            │
│       │                                                         │
│       ▼                                                         │
│  5. [仿真] 运行纯位置模式仿真                                    │
│       │  python quad_simulation_test.py --input "xxx.csv" --full│
│       │                                                         │
│       ▼                                                         │
│  6. [分析] 查看输出报告和可视化                                  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 快速测试（不修改STM32代码）

如果不想修改STM32代码，也可以直接运行仿真脚本，它会基于CAN数据中的命令模拟电机响应：

```bash
# 纯位置模式仿真
python quad_simulation_test.py --input "1-6电机测试分析仪实际.csv" --full
```

---

## 四、力位混合模式的足端力模拟

### 4.1 模拟原理

由于没有真实的足端力传感器，仿真系统使用以下模型：

```
支撑相(stance):  腿接触地面，受力 > 阈值(2N)
    - 基于关节角度估算受力
    - F ≈ k * sin(关节角度) * 基础力

摆动相(swing):   腿在空中移动，受力 ≈ 0
    - 添加少量高斯噪声模拟传感器噪声
```

### 4.2 支持的步态类型

- **trot ( trot步态)**: 对角腿同步
- **walk (行走步态)**: 依次迈腿
- **gallop ( gallop步态)**: 奔跑

---

## 五、输出数据分析

### 5.1 报告指标说明

| 指标 | 说明 | 评估标准 |
|------|------|----------|
| 跟踪质量 | Overall tracking quality | EXCELLENT/GOOD/FAIR/POOR |
| 最大位置误差 | Maximum position error | <1°优秀 <5°良好 <10°一般 |
| 平均位置误差 | Average position error | 同上 |
| RMS误差 | Root mean square error | 越接近0越好 |

### 5.2 稳定性判断

- **EXCELLENT**: 所有电机跟踪误差 <1°
- **GOOD**: 所有电机跟踪误差 <5°
- **FAIR**: 存在部分电机误差 5-10°
- **POOR**: 存在电机误差 >10°

---

## 六、注意事项

1. **超时禁用仅用于测试**：正式运行时请恢复超时设置
2. **CANFD数据格式**：确保CSV包含CAN ID、时间戳、数据字段
3. **波形图需要matplotlib**: 如未安装请运行 `pip install matplotlib numpy`
4. **3D动画需要较多样本**: 数据点太少可能显示不佳

---

## 七、故障排查

### 问题1: 解析失败

```
[错误] 文件不存在
```

**解决**: 检查CSV文件路径是否正确

### 问题2: 无有效帧

```
[警告] 未解析到有效帧
```

**解决**: 检查CSV格式是否与解析器兼容，或尝试其他CSV文件

### 问题3: 波形图不显示

**解决**: 确保安装了matplotlib
```bash
pip install matplotlib numpy
```

### 问题4: 仿真时间过长

**解决**: 减少数据量或使用 `--analyze` 仅做分析
