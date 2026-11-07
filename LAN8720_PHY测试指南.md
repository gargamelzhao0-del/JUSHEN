# LAN8720 PHY 和 TCP 通信硬件测试指南

## 一、问题诊断

从串口日志显示的问题：
```
[LAN8720] PHY ID: 0xFFFFFFFF  ← MDIO/MDC通信失败
[LAN8720] WARNING: Unexpected PHY ID! Expected 0x0007C0Fx
```

**PHY ID返回0xFFFFFFFF表示PHY芯片完全无响应**，这是最底层的通信问题。

---

## 二、硬件连接检查清单

### 2.1 RMII引脚对照表

请对照你的原理图检查以下连接：

| STM32H7引脚 | LAN8720引脚 | 功能 | 检查方法 |
|------------|-------------|------|----------|
| **PA1** | Pin 2 (nINT/REFCLKO) | **REF_CLK 50MHz** | ⚠️ **最关键！用示波器验证** |
| **PA2** | Pin 5 (MDIO) | 管理数据 | 万用表测连通 |
| **PC1** | Pin 6 (MDC) | 管理时钟 | 万用表测连通 |
| **PA7** | Pin 8 (CRS_DV) | 载波侦测/数据有效 | 万用表测连通 |
| **PC4** | Pin 9 (RXD0) | 接收数据0 | 万用表测连通 |
| **PC5** | Pin 10 (RXD1) | 接收数据1 | 万用表测连通 |
| **PB11** | Pin 11 (TX_EN) | 发送使能 | 万用表测连通 |
| **PB12** | Pin 12 (TXD0) | 发送数据0 | 万用表测连通 |
| **PB13** | Pin 13 (TXD1) | 发送数据1 | 万用表测连通 |

### 2.2 电源和复位检查

| 检查项 | LAN8720引脚 | 预期值 | 测试工具 |
|--------|-------------|--------|----------|
| VDD (主电源) | Pin 1, 14 | 3.3V ±5% | 万用表 |
| GND (接地) | Pin 7, 15 | 0V | 万用表 |
| nRST (复位) | Pin 3 | 高电平(3.3V) | 万用表 |

### 2.3 PHY地址配置

LAN8720的PHY地址由 **RXER/PHYAD0 (Pin 16)** 决定：
- 接GND → PHY地址 = **0x00**
- 接VDD → PHY地址 = **0x01**

当前代码使用固定地址 `0x01`，请确认你的原理图配置。

---

## 三、REF_CLK验证（最关键）

### 3.1 为什么REF_CLK最重要

LAN8720需要50MHz参考时钟才能工作。**没有时钟，PHY完全不响应MDIO指令！**

### 3.2 时钟来源方式

**方式1：LAN8720内部振荡器（推荐）**
- LAN8720 XTAL1/XTAL2 (Pin 4, 5) 接25MHz晶振
- PHY内部振荡器产生50MHz时钟，从 Pin 2 (nINT/REFCLKO) 输出
- 连接PA1到Pin 2

**方式2：外部时钟输入**
- 外部50MHz时钟源输入到 XTAL1/CLKIN (Pin 4)

### 3.3 示波器验证步骤

1. 探头连接 **PA1** (STM32) 或 **LAN8720 Pin 2**
2. 预期信号：
   - 频率：50MHz
   - 幅度：约3.3V
   - 波形：方波

如果**无50MHz信号**，PHY无法工作，必须先解决时钟问题。

---

## 四、软件诊断测试

### 4.1 使用PHY诊断测试程序

已创建诊断测试文件：
- [phy_diagnostic_test.h](Core/Inc/phy_diagnostic_test.h)
- [phy_diagnostic_test.c](Core/Src/phy_diagnostic_test.c)

### 4.2 测试步骤

**Step 1：在main.c或freertos.c中调用诊断函数**

```c
#include "phy_diagnostic_test.h"

// 在系统启动后调用
PHY_TestResult_t result = PHY_RunFullDiagnostic();
```

**Step 2：查看串口输出，诊断程序会执行以下测试：**

1. ETH初始化状态检查
2. MDIO时钟配置检查
3. PHY地址扫描（自动扫描0x00-0x1F）
4. PHY ID验证
5. PHY寄存器读取（BCR/BSR）
6. Link状态检查

### 4.3 测试结果解读

| 测试结果 | 说明 | 下一步 |
|----------|------|--------|
| PHY_TEST_OK | PHY正常，Link UP | 可进行TCP测试 |
| PHY_TEST_NO_RESPONSE | 无PHY响应 | 检查MDIO/MDC/REF_CLK |
| PHY_TEST_WRONG_ID | PHY ID不匹配 | 可能是不同型号PHY |
| PHY_TEST_LINK_DOWN | PHY正常，但无网线 | 插网线后测试TCP |

---

## 五、TCP通信测试（PHY正常后）

### 5.1 基础网络测试

**Ping测试：**
```bash
ping 192.168.1.100  # 你的板子IP地址
```

### 5.2 TCP连接测试

**使用Python测试脚本：**
```python
import socket
import time

# 连接板子
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.1.100', 9999))
sock.settimeout(5)

# 发送测试数据
test_data = bytes([0x01, 0x02, 0x03, 0x04])
sock.send(test_data)

# 接收响应
response = sock.recv(1024)
print(f"Received: {response}")

sock.close()
```

### 5.3 使用NetAssist工具

1. 打开NetAssist网络调试助手
2. TCP Client模式
3. 连接到板子IP:端口 (192.168.1.100:9999)
4. 发送数据观察响应

---

## 六、常见问题排查

### 6.1 PHY无响应 (ID=0xFFFFFFFF)

**可能原因：**
1. ❌ REF_CLK无50MHz时钟（最常见）
2. ❌ MDIO/MDC连接错误
3. ❌ PHY电源未供电
4. ❌ nRST被拉低（复位状态）
5. ❌ PHY地址配置错误

**解决步骤：**
```
1. 示波器检查PA1是否有50MHz时钟 → 无时钟则查晶振/时钟电路
2. 万用表检查PHY VDD=3.3V → 无电压则查电源电路
3. 万用表检查MDIO/MDC连通 → 断线则补焊
4. 尝试PHY地址0x00和0x01
```

### 6.2 PHY正常但Link DOWN

**可能原因：**
1. 网线未连接
2. 网线损坏
3. 对端设备（路由器/交换机）问题
4. PHY自动协商失败

**解决步骤：**
```
1. 插网线，观察PHY LED指示灯
2. 尝试直连电脑（不用路由器）
3. 强制设置速度/双工模式
```

### 6.3 PHY正常但无法Ping

**可能原因：**
1. IP地址配置错误
2. MAC地址过滤
3. 网线问题（RX/TX方向错误）
4. DMA描述符配置错误

**解决步骤：**
```
1. 检查串口输出的IP地址是否正确
2. 检查电脑IP在同一网段
3. 交叉网线TX/RX连接
```

---

## 七、新板子测试流程图

```
开始测试
    ↓
[1] 万用表测PHY VDD=3.3V → 不通过 → 检查电源电路
    ↓ 通过
[2] 示波器测PA1=50MHz → 不通过 → 检查晶振/时钟电路
    ↓ 通过
[3] 调用PHY_RunFullDiagnostic() → 查看串口输出
    ↓
    ├─ PHY_TEST_NO_RESPONSE → 检查MDIO/MDC连接
    ├─ PHY_TEST_WRONG_ID → 可能是其他PHY型号，继续测试
    └─ PHY_TEST_OK 或 PHY_TEST_LINK_DOWN
    ↓
[4] 插网线 → Link UP
    ↓
[5] Ping测试 → 不通过 → 检查IP配置
    ↓ 通过
[6] TCP连接测试 → 完成
```

---

## 八、硬件测试报告模板

测试完成后请填写：

```
测试板号: ___________
测试日期: ___________
测试人员: ___________

【电源测试】
- PHY VDD电压: _____ V (正常: 3.3V)
- PHY GND连通: _____ (正常: OK)

【时钟测试】
- PA1时钟频率: _____ MHz (正常: 50MHz)
- 时钟幅度: _____ V (正常: ~3.3V)

【MDIO测试】
- MDIO连通: _____ (正常: OK)
- MDC连通: _____ (正常: OK)

【PHY ID测试】
- PHY地址: 0x__
- PHY ID: 0x________ (正常: 0x0007C0Fx)

【Link测试】
- Link状态: _____ (正常: UP)
- LED指示: _____ (正常: 绿灯亮)

【TCP测试】
- Ping结果: _____
- TCP连接: _____
- 数据传输: _____

问题记录:
_______________________________________
_______________________________________
```