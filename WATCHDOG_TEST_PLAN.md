# 看门狗保护系统测试计划

## 测试目标

验证看门狗系统的两层保护机制：
1. **软件任务监控** - 检测任务超时（3秒）
2. **硬件IWDG复位** - 系统死机复位（5秒）

---

## ⚠️ 测试前准备

### 1. 确认IWDG已启动

查看启动日志，应该看到：
```
[Watchdog] Waiting 5s for system stabilization...
[Watchdog] Starting IWDG (cannot be stopped once started)...
[Watchdog] IWDG started successfully
[Watchdog] ⚠️  System MUST feed IWDG within 5000 ms to prevent reset!
```

### 2. 调试器注意事项

**重要**：IWDG在调试模式下仍然运行！

- ❌ **不要使用断点**：断点会暂停程序，超过5秒IWDG会复位系统
- ✅ **使用日志观察**：通过串口日志监控系统状态
- ✅ **如需调试**：暂时注释掉 `watchdog_start_iwdg()` 这一行

### 3. 备份当前代码

测试前建议备份代码，以防意外。

---

## 测试用例

### 测试1：正常运行测试（基线）

**目的**：验证系统正常运行时，IWDG不会触发复位

**步骤**：
1. 编译并烧录代码
2. 连接串口，查看日志
3. 观察系统运行至少2分钟

**预期结果**：
- ✅ 每30秒看到看门狗统计信息
- ✅ 所有任务状态为 OK
- ✅ IWDG Feed Count 持续增加（每秒+1）
- ✅ 系统不会复位

**日志示例**：
```
[    30.000] [Watchdog] Check Cycles: 30
[    30.000] [Watchdog] IWDG Feed Count: 30 (last: 0 ms ago)
[    30.000] [Watchdog] Total Timeouts: 0
[    30.000] [Watchdog]   DefaultTask: OK (heartbeats=3000, ...)
[    30.000] [Watchdog]   MotorControlLoop: OK (heartbeats=300, ...)
[    30.000] [Watchdog]   TCPSend: OK (heartbeats=3000, ...)
[    30.000] [Watchdog]   FDCANRx: OK (heartbeats=500, ...)
[    30.000] [Watchdog]   Watchdog: OK (heartbeats=30, ...)
```

**通过标准**：
- 无 WARNING 或 TIMEOUT 日志
- 系统持续运行，无复位

---

### 测试2：软件任务超时检测

**目的**：验证软件看门狗能检测到任务卡死

**测试2.1：模拟电机任务卡死（10秒）**

#### 修改代码

在 [freertos.c](Core/Src/freertos.c) 的 `MotorControlLoopTask` 中添加测试代码：

```c
void MotorControlLoopTask(void *argument)
{
    uint32_t loop_count = 0;
    // ... 初始化代码 ...

    /* 测试开关 - 默认关闭 */
    bool test_task_hang = false;  // ⭐ 添加测试标志

    for(;;)
    {
        /* 等待定时器信号量 */
        osSemaphoreAcquire(timer_sem, osWaitForever);

        loop_count++;

        /* ===== 测试代码：模拟任务卡死 ===== */
        if (loop_count == 100) {  /* 运行10秒后触发 */
            test_task_hang = true;
        }

        if (test_task_hang) {
            LOG_WARN("[MotorCtrl] ⚠️  TEST: Simulating task hang for 10 seconds...");
            osDelay(10000);  /* 卡死10秒 */
            test_task_hang = false;
            LOG_INFO("[MotorCtrl] TEST: Task resumed");
        }
        /* ===== 测试代码结束 ===== */

        /* 正常控制逻辑 */
        // ...

        watchdog_feed_task(WDG_TASK_MOTOR_CTRL);
    }
}
```

#### 预期结果

**3秒后**（任务超时）:
```
[Watchdog] Task 'MotorControlLoop' heartbeat slow (1800 ms since last)
```

**超过3秒后**:
```
[Watchdog] ⚠️  Task 'MotorControlLoop' TIMEOUT! (no heartbeat for 3500 ms)
```

**5秒后**（IWDG触发）:
```
系统复位...

[重启后]
[Main] ⚠️  Last reset was caused by watchdog!
```

**通过标准**：
- ✅ 软件看门狗检测到任务超时（3秒）
- ✅ IWDG触发系统复位（5秒）
- ✅ 重启后检测到看门狗复位原因

---

### 测试3：看门狗任务自身卡死

**目的**：验证看门狗任务卡死时，IWDG会复位系统

**测试方法**：

在 [watchdog.c](Core/Src/watchdog.c) 的 `StartWatchdogTask` 中添加测试代码：

```c
void StartWatchdogTask(void *argument)
{
    // ... 初始化代码 ...

    /* 统计打印计数器 */
    uint32_t stats_print_counter = 0;
    bool test_watchdog_hang = false;  // ⭐ 测试标志

    /* 主循环 */
    while (1) {
        /* 自身心跳 */
        watchdog_feed_task(WDG_TASK_WATCHDOG);

        /* 检查所有任务健康状态 */
        watchdog_check_all_tasks();

        /* ===== 测试代码：模拟看门狗任务卡死 ===== */
        if (stats_print_counter == 6) {  /* 60秒后触发 */
            test_watchdog_hang = true;
        }

        if (test_watchdog_hang) {
            LOG_ERROR("[Watchdog] ⚠️  TEST: Simulating watchdog hang (IWDG will reset system in 5s)...");
            while(1) {
                /* 无限循环，不喂IWDG */
                osDelay(100);
            }
        }
        /* ===== 测试代码结束 ===== */

        /* 喂硬件看门狗 */
        watchdog_feed_iwdg();

        // ...
    }
}
```

#### 预期结果

**触发后立即**:
```
[Watchdog] ⚠️  TEST: Simulating watchdog hang (IWDG will reset system in 5s)...
```

**5秒后**:
```
系统复位...（无日志，因为CPU已卡死）

[重启后]
[Main] ⚠️  Last reset was caused by watchdog!
```

**通过标准**：
- ✅ IWDG在5秒后触发复位
- ✅ 系统成功重启
- ✅ 检测到看门狗复位原因

---

### 测试4：高频任务压力测试

**目的**：验证高负载下看门狗仍能正常工作

**步骤**：
1. 启用电机发送（100Hz电机命令）
2. 启用TCP发送（100Hz心跳）
3. 观察系统运行30分钟

**监控指标**：
```c
/* 每5分钟检查一次 */
[Watchdog] IWDG Feed Count: 300 (last: 0 ms ago)  ✅ 应为0-1000ms
[Watchdog] Total Timeouts: 0                      ✅ 应为0
[Watchdog]   MotorControlLoop: OK                 ✅ last < 500ms
[Watchdog]   TCPSend: OK                          ✅ last < 100ms
```

**通过标准**：
- ✅ 无任务超时
- ✅ IWDG正常喂狗（last < 1000ms）
- ✅ 系统稳定运行，无复位

---

### 测试5：任务恢复测试

**目的**：验证任务从超时状态恢复后，看门狗能检测到

**测试方法**：

模拟任务短暂卡死后恢复：

```c
void MotorControlLoopTask(void *argument)
{
    for(;;)
    {
        loop_count++;

        /* 测试：短暂卡死2秒（不足以触发IWDG） */
        if (loop_count == 50) {
            LOG_WARN("[Test] Task hang for 2s...");
            osDelay(2000);
            LOG_INFO("[Test] Task resumed");
        }

        /* 继续正常工作 */
        motor_sync_send_all_position(...);
        watchdog_feed_task(WDG_TASK_MOTOR_CTRL);
    }
}
```

#### 预期结果

**卡死时**（1.5秒后）:
```
[Watchdog] Task 'MotorControlLoop' heartbeat slow (1800 ms since last)
```

**恢复后**（下一次检查）:
```
[Watchdog] Task 'MotorControlLoop' recovered from timeout
[Watchdog]   MotorControlLoop: OK (heartbeats=60, timeouts=1, ...)
```

**通过标准**：
- ✅ 检测到WARNING或TIMEOUT
- ✅ 恢复后状态变为OK
- ✅ timeout_count增加1

---

### 测试6：禁用任务监控测试

**目的**：验证禁用监控后，任务超时不会被检测

**步骤**：

1. 在初始化时禁用电机任务监控：
```c
void MX_FREERTOS_Init(void) {
    /* 初始化看门狗 */
    watchdog_init();

    /* 禁用电机任务监控（测试用）*/
    watchdog_set_task_enabled(WDG_TASK_MOTOR_CTRL, false);
}
```

2. 运行测试2.1（模拟电机任务卡死）

#### 预期结果

**电机任务卡死时**:
```
[Watchdog]   MotorControlLoop: DISABLED  ✅ 不会检测超时
```

**5秒后仍会触发IWDG复位**（因为看门狗任务也被阻塞）

**通过标准**：
- ✅ 软件监控显示 DISABLED
- ✅ 不会输出TIMEOUT日志
- ✅ IWDG仍会触发复位（如果影响到看门狗任务）

---

## 测试记录表

| 测试用例 | 执行日期 | 结果 | 备注 |
|---------|---------|------|------|
| 测试1: 正常运行 | ______ | ⬜ 通过 / ⬜ 失败 | |
| 测试2.1: 电机任务超时 | ______ | ⬜ 通过 / ⬜ 失败 | |
| 测试3: 看门狗卡死 | ______ | ⬜ 通过 / ⬜ 失败 | |
| 测试4: 高负载压力 | ______ | ⬜ 通过 / ⬜ 失败 | |
| 测试5: 任务恢复 | ______ | ⬜ 通过 / ⬜ 失败 | |
| 测试6: 禁用监控 | ______ | ⬜ 通过 / ⬜ 失败 | |

---

## 故障排查

### 问题1: 系统频繁复位

**可能原因**：
- 看门狗任务被高优先级任务饿死
- 任务执行时间过长（超过5秒）
- 中断处理时间过长

**诊断**：
```c
/* 在看门狗任务中添加 */
uint32_t last_feed_time = HAL_GetTick();
watchdog_feed_iwdg();
uint32_t feed_interval = HAL_GetTick() - last_feed_time;

if (feed_interval > 2000) {
    LOG_WARN("[Watchdog] Feed interval too long: %lu ms", feed_interval);
}
```

**解决方案**：
- 提高看门狗任务优先级（不推荐）
- 降低其他任务执行时间
- 增加IWDG超时时间（改为10秒）

### 问题2: 任务超时未检测到

**可能原因**：
- 任务监控被禁用
- 心跳上报位置错误
- 互斥锁死锁

**诊断**：
```c
/* 检查任务是否启用 */
WDG_TaskInfo_t info;
watchdog_get_task_info(WDG_TASK_MOTOR_CTRL, &info);
printf("Enabled: %d, Status: %s\n", info.enabled,
       watchdog_get_health_string(info.status));
```

### 问题3: IWDG未启动

**症状**：系统卡死但不复位

**检查**：
- 查看启动日志是否有 "IWDG started successfully"
- 确认 `watchdog_start_iwdg()` 没有被注释

**修复**：
- 取消注释 watchdog.c:436 的 `watchdog_start_iwdg();`

---

## 性能验证

### CPU占用测试

**目标**：看门狗系统CPU占用 < 0.5%

**测试方法**：
1. 使用FreeRTOS任务统计功能
2. 计算看门狗任务运行时间占比

```c
/* 在看门狗任务中 */
static uint32_t total_runtime = 0;
uint32_t start_time = HAL_GetTick();

/* 执行看门狗逻辑 */
watchdog_check_all_tasks();
watchdog_feed_iwdg();

uint32_t runtime = HAL_GetTick() - start_time;
total_runtime += runtime;

if (stats_print_counter == 0) {
    float cpu_usage = (float)total_runtime / (float)(HAL_GetTick()) * 100.0f;
    LOG_INFO("[Watchdog] CPU usage: %.2f%%", cpu_usage);
}
```

**通过标准**：
- ✅ CPU占用 < 0.5%
- ✅ 单次检查耗时 < 5ms

### 内存占用测试

**目标**：看门狗系统内存占用 < 2KB

**验证**：
```c
/* 查看堆内存 */
size_t heap_before = xPortGetFreeHeapSize();
watchdog_init();
size_t heap_after = xPortGetFreeHeapSize();

printf("Watchdog memory: %u bytes\n", heap_before - heap_after);
```

**通过标准**：
- ✅ 动态内存分配 < 500 bytes
- ✅ 任务堆栈 = 1KB
- ✅ 总内存 < 2KB

---

## 安全建议

### 生产环境部署

1. **确认所有任务都上报心跳**
   - 检查5个关键任务的心跳代码
   - 验证心跳频率合理（< 超时时间）

2. **调整超时时间**
   - 根据实际任务周期调整 `WDG_TASK_TIMEOUT_MS`
   - 建议设为任务周期的3-5倍

3. **启用IWDG**
   - 确保 `watchdog_start_iwdg()` 未被注释
   - 生产环境**必须启用**

4. **复位日志记录**
   - 在FLASH中记录看门狗复位次数
   - 分析复位原因，优化系统稳定性

### 调试环境配置

1. **禁用IWDG**
   - 注释掉 `watchdog_start_iwdg()`
   - 仅使用软件任务监控

2. **增加超时时间**
   ```c
   #define WDG_TASK_TIMEOUT_MS  10000  /* 调试时设为10秒 */
   ```

3. **详细日志**
   - 启用DEBUG级别日志
   - 打印每次心跳（仅调试用）

---

## 总结

### 测试清单

- [ ] 测试1：正常运行（2分钟无复位）
- [ ] 测试2：任务超时检测（3秒检测到）
- [ ] 测试3：IWDG复位（5秒触发）
- [ ] 测试4：高负载稳定性（30分钟）
- [ ] 测试5：任务恢复检测
- [ ] 测试6：禁用监控功能
- [ ] 性能验证：CPU < 0.5%, 内存 < 2KB
- [ ] 复位原因检测正常

### 验收标准

✅ **所有测试用例通过**
✅ **性能指标达标**
✅ **无误报超时**
✅ **复位保护有效**

---

**测试负责人**: __________
**测试日期**: __________
**系统版本**: __________

**最后更新**: 2026-02-11
