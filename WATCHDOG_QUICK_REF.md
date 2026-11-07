# 看门狗测试快速参考

## 🚀 快速开始

### 1. 编译并烧录
```bash
# 确保代码已保存并编译
make clean && make
# 烧录到板子
```

### 2. 连接串口
- 波特率：115200
- 数据位：8
- 停止位：1
- 校验：无

### 3. 查看启动日志

应该看到：
```
[Watchdog] Watchdog Task Started
[Watchdog] Waiting 5s for system stabilization...
[Watchdog] Starting IWDG (cannot be stopped once started)...
[Watchdog] IWDG started successfully
[Watchdog] ⚠️  System MUST feed IWDG within 5000 ms to prevent reset!
```

---

## 🎮 串口测试命令

### 看门狗命令

| 按键 | 功能 | 说明 |
|-----|------|------|
| `W` 或 `w` | 打印统计信息 | 显示所有任务健康状态 |
| `1` | 电机任务卡死测试 | 卡死10秒，测试软件监控 |
| `2` | TCP任务卡死测试 | 卡死5秒，测试超时检测 |
| `3` | 看门狗卡死测试 | 永久卡死，触发IWDG复位 |
| `0` | 显示帮助 | 显示测试命令列表 |

### 电机测试命令（保留原有功能）

| 按键 | 功能 |
|-----|------|
| `N` 或 `n` | 发送下一条测试 |
| `R` 或 `r` | 重发当前测试 |
| `S` 或 `s` | 跳过当前测试 |
| `P` 或 `p` | 显示进度 |
| `H` 或 `h` 或 `?` | 显示帮助 |
| `Q` 或 `q` | 退出测试 |

---

## 📋 测试流程

### 测试1：基线测试（正常运行）

1. **启动系统**，等待初始化完成
2. **按 `W`**，查看统计信息
3. **观察30秒**，应该看到周期性统计输出
4. **再次按 `W`**，确认所有任务 OK

**预期结果**：
```
[Watchdog] Check Cycles: 60
[Watchdog] IWDG Feed Count: 60 (last: 0 ms ago)
[Watchdog] Total Timeouts: 0
[Watchdog]   DefaultTask: OK (heartbeats=6000, timeouts=0, last=0 ms ago)
[Watchdog]   MotorControlLoop: OK (heartbeats=600, timeouts=0, last=50 ms ago)
[Watchdog]   TCPSend: OK (heartbeats=6000, timeouts=0, last=0 ms ago)
[Watchdog]   FDCANRx: OK (heartbeats=600, timeouts=0, last=5 ms ago)
[Watchdog]   Watchdog: OK (heartbeats=60, timeouts=0, last=0 ms ago)
```

✅ **通过**：所有任务 OK，无超时

---

### 测试2：软件任务超时检测

1. **按 `1`** - 触发电机任务卡死10秒
2. **观察日志**

**预期输出**：
```
[TEST] Triggering MotorCtrl hang (10s)...
[Watchdog] ⚠️  TEST: Triggering MotorControlLoop hang for 10000 ms
[MotorCtrl] ⚠️  TEST: Simulating task hang for 10000 ms

[3秒后]
[Watchdog] Task 'MotorControlLoop' heartbeat slow (1800 ms since last)

[3秒后]
[Watchdog] ⚠️  Task 'MotorControlLoop' TIMEOUT! (no heartbeat for 3500 ms)

[5秒后]
系统复位...

[重启后]
[Main] ⚠️  Last reset was caused by watchdog!
```

✅ **通过**：
- 3秒检测到WARNING
- 5秒触发IWDG复位
- 重启后检测到看门狗复位

---

### 测试3：看门狗自身保护

1. **按 `3`** - 触发看门狗任务卡死
2. **观察系统**

**预期输出**：
```
[TEST] Triggering Watchdog hang (IWDG will reset in 5s)...
[Watchdog] ⚠️  TEST: Triggering Watchdog hang for 0 ms
[Watchdog] ⚠️  TEST: Simulating watchdog hang...
[Watchdog] TEST: Infinite hang - IWDG will reset system in 5s

[5秒后]
系统复位...（无日志，CPU已卡死）

[重启后]
[Main] ⚠️  Last reset was caused by watchdog!
```

✅ **通过**：
- 5秒后系统复位
- 重启后检测到看门狗复位

---

### 测试4：任务恢复检测

1. **按 `2`** - 触发TCP任务卡死5秒（<10秒，不触发IWDG）
2. **观察日志**

**预期输出**：
```
[TEST] Triggering TCPSend hang (5s)...

[3秒后]
[Watchdog] ⚠️  Task 'TCPSend' TIMEOUT! (no heartbeat for 3500 ms)

[5秒后，任务恢复]
[Watchdog] Task 'TCPSend' recovered from timeout
[Watchdog]   TCPSend: OK (heartbeats=500, timeouts=1, last=10 ms ago)
```

✅ **通过**：
- 检测到超时
- 任务恢复后状态变为OK
- timeouts计数增加

---

## ⚠️ 注意事项

### IWDG已启用

- ✅ **不要使用调试器断点**（会触发复位）
- ✅ **使用串口日志观察**
- ❌ **不要在中断中长时间阻塞**

### 如需调试

在 [watchdog.c](Core/Src/watchdog.c):436 注释掉：
```c
/* 启动IWDG（5秒超时，必须定期喂狗）*/
// watchdog_start_iwdg();  // 调试时注释此行
```

### 测试后恢复

测试完成后，移除测试代码：
1. 注释掉 `MotorControlLoopTask` 中的测试检查代码
2. 删除 `StartDefaultTask` 中的测试命令（可选）

---

## 📊 性能验证

### CPU占用

查看看门狗统计：
```
[Watchdog] Check Cycles: 300
[Watchdog] IWDG Feed Count: 300
```

300秒运行300次检查 = 1次/秒 ✅

### 内存占用

查看堆内存：
```
[MotorCtrl]   Heap: 125000 bytes free
```

看门狗启动前后堆内存差 < 2KB ✅

---

## 🔍 故障排查

### 问题：系统频繁复位

**诊断**：
- 按 `W` 查看统计信息
- 检查 IWDG Feed Count 是否正常增长
- 查看是否有任务长时间卡死

**解决**：
- 增加IWDG超时时间（改为10秒）
- 检查高优先级任务是否过长

### 问题：任务超时未检测

**诊断**：
- 按 `W` 查看任务是否 ENABLED
- 检查心跳上报代码是否正确

**解决**：
- 确认 `watchdog_feed_task()` 被调用
- 检查互斥锁是否死锁

### 问题：IWDG未启动

**诊断**：
- 查看启动日志是否有 "IWDG started successfully"

**解决**：
- 确认 watchdog.c:436 未被注释

---

## ✅ 验收标准

- [ ] 测试1：正常运行60秒无复位
- [ ] 测试2：电机卡死触发超时和IWDG复位
- [ ] 测试3：看门狗卡死触发IWDG复位
- [ ] 测试4：任务恢复被检测
- [ ] CPU占用 < 0.5%
- [ ] 内存占用 < 2KB
- [ ] 复位原因检测正常

---

**快速测试序列**（5分钟）：
1. 启动系统 → 按 `W` → 确认所有OK
2. 按 `1` → 等待复位 → 确认检测到看门狗复位
3. 按 `2` → 观察恢复 → 按 `W` → 确认timeouts=1
4. 按 `3` → 等待复位 → 确认检测到看门狗复位

**最后更新**: 2026-02-11
