# 看门狗保护系统使用指南

## 系统概述

本系统实现了**双重看门狗保护机制**：
1. **硬件看门狗（IWDG）**: STM32H7 独立看门狗，5秒超时自动复位系统
2. **软件任务监控**: 监控关键任务的心跳，检测任务卡死或饿死

---

## 架构设计

### 1. 硬件看门狗（IWDG）

**配置参数**:
- 时钟源: LSI (32kHz)
- 预分频: /128 → 250Hz
- 重载值: 1250
- 超时时间: **5秒** (1250 / 250Hz)

**工作原理**:
```
启动IWDG → 必须在5秒内喂狗 → 否则系统复位
```

**注意事项**:
- ⚠️ IWDG一旦启动就**无法停止**
- 系统稳定后再启动（建议在所有任务初始化完成后）
- 看门狗任务每1秒自动喂一次IWDG

### 2. 软件任务监控

**监控的任务**:
```c
WDG_TASK_DEFAULT       // DefaultTask (LwIP主任务)
WDG_TASK_MOTOR_CTRL    // MotorControlLoopTask (电机控制)
WDG_TASK_TCP_SEND      // TCPSendTask (TCP发送)
WDG_TASK_FDCAN_RX      // FDCANRxTask (CAN接收)
WDG_TASK_WATCHDOG      // WatchdogTask (看门狗自身)
```

**超时配置**:
- 任务心跳超时: **3秒**
- 看门狗检查间隔: **1秒**
- 警告阈值: 1.5秒（超时的一半）

**健康状态**:
```c
WDG_HEALTH_OK           // 任务健康
WDG_HEALTH_WARNING      // 心跳较慢（超过1.5秒）
WDG_HEALTH_TIMEOUT      // 任务超时（超过3秒）
WDG_HEALTH_NOT_STARTED  // 任务未启动
```

---

## API函数

### 初始化和控制

#### 1. 初始化看门狗系统
```c
int watchdog_init(void);
```
- **功能**: 初始化看门狗系统（IWDG配置 + 任务监控表）
- **返回值**: 0=成功, -1=失败
- **调用时机**: 在FreeRTOS启动前，`MX_FREERTOS_Init()`中

#### 2. 启动硬件看门狗
```c
int watchdog_start_iwdg(void);
```
- **功能**: 启动IWDG（不可逆操作）
- **返回值**: 0=成功, -1=失败
- **调用时机**: 系统完全稳定后（可选，也可由看门狗任务自动启动）

**示例**:
```c
void MX_FREERTOS_Init(void) {
    /* ... 其他初始化 ... */

    /* 初始化看门狗系统 */
    if (watchdog_init() != 0) {
        LOG_ERROR("[Main] Watchdog init failed!");
        Error_Handler();
    }

    /* 检查上次复位原因 */
    if (watchdog_check_reset_reason()) {
        LOG_ERROR("[Main] ⚠️  Last reset was caused by watchdog!");
    }
}
```

#### 3. 喂硬件看门狗
```c
void watchdog_feed_iwdg(void);
```
- **功能**: 刷新IWDG计数器，重置超时时间
- **调用频率**: 每1秒（由看门狗任务自动调用）

#### 4. 检查系统重启原因
```c
bool watchdog_check_reset_reason(void);
```
- **功能**: 检查上次系统复位是否由IWDG触发
- **返回值**: true=看门狗复位, false=其他原因
- **作用**: 用于故障诊断和统计

---

### 任务心跳监控

#### 1. 注册任务心跳
```c
void watchdog_feed_task(WDG_TaskID_t task_id);
```
- **功能**: 任务定期调用此函数，上报心跳
- **参数**: 任务ID (WDG_TASK_DEFAULT, WDG_TASK_MOTOR_CTRL等)
- **调用频率**: 每个任务循环至少调用一次（建议每100ms-1s）

**示例**:
```c
void MotorControlLoopTask(void *argument)
{
    for(;;)
    {
        /* 等待定时器触发 */
        osSemaphoreAcquire(timer_sem, osWaitForever);

        /* 执行电机控制逻辑 */
        motor_sync_send_all_position(...);

        /* 上报心跳 */
        watchdog_feed_task(WDG_TASK_MOTOR_CTRL);

        osDelay(100);
    }
}
```

#### 2. 使能/禁用任务监控
```c
void watchdog_set_task_enabled(WDG_TaskID_t task_id, bool enabled);
```
- **功能**: 动态启用或禁用某个任务的监控
- **参数**:
  - `task_id`: 任务ID
  - `enabled`: true=启用, false=禁用
- **使用场景**: 某些任务在特定阶段不需要监控时

**示例**:
```c
/* 在调试时暂时禁用电机任务监控 */
watchdog_set_task_enabled(WDG_TASK_MOTOR_CTRL, false);

/* 测试完成后重新启用 */
watchdog_set_task_enabled(WDG_TASK_MOTOR_CTRL, true);
```

#### 3. 获取任务健康状态
```c
WDG_HealthStatus_t watchdog_get_task_health(WDG_TaskID_t task_id);
```
- **功能**: 查询任务当前的健康状态
- **返回值**: WDG_HEALTH_OK / WARNING / TIMEOUT / NOT_STARTED

**示例**:
```c
WDG_HealthStatus_t status = watchdog_get_task_health(WDG_TASK_TCP_SEND);
if (status == WDG_HEALTH_TIMEOUT) {
    LOG_ERROR("TCP任务已超时，可能卡死！");
}
```

#### 4. 获取任务详细信息
```c
int watchdog_get_task_info(WDG_TaskID_t task_id, WDG_TaskInfo_t *info);
```
- **功能**: 获取任务的详细监控信息
- **返回值**: 0=成功, -1=失败
- **输出信息**:
  - `task_name`: 任务名称
  - `last_heartbeat_ms`: 上次心跳时间戳
  - `heartbeat_count`: 心跳总次数
  - `timeout_count`: 超时次数
  - `status`: 健康状态
  - `enabled`: 是否启用监控

**示例**:
```c
WDG_TaskInfo_t info;
if (watchdog_get_task_info(WDG_TASK_MOTOR_CTRL, &info) == 0) {
    printf("任务: %s\n", info.task_name);
    printf("心跳次数: %u\n", info.heartbeat_count);
    printf("超时次数: %u\n", info.timeout_count);
    printf("状态: %s\n", watchdog_get_health_string(info.status));
}
```

---

### 统计和调试

#### 1. 打印看门狗统计信息
```c
void watchdog_print_stats(void);
```
- **功能**: 打印完整的看门狗统计信息（所有任务状态）
- **调用频率**: 建议每30秒打印一次（看门狗任务自动执行）

**输出示例**:
```
[Watchdog] ==================== Statistics ====================
[Watchdog] Check Cycles: 300
[Watchdog] IWDG Feed Count: 300 (last: 1000 ms ago)
[Watchdog] Total Timeouts: 0
[Watchdog] --- Task Health Status ---
[Watchdog]   DefaultTask: OK (heartbeats=3000, timeouts=0, last=100 ms ago)
[Watchdog]   MotorControlLoop: OK (heartbeats=1000, timeouts=0, last=100 ms ago)
[Watchdog]   TCPSend: OK (heartbeats=30000, timeouts=0, last=10 ms ago)
[Watchdog]   FDCANRx: OK (heartbeats=5000, timeouts=0, last=50 ms ago)
[Watchdog]   Watchdog: OK (heartbeats=300, timeouts=0, last=1000 ms ago)
[Watchdog] ====================================================
```

#### 2. 工具函数
```c
const char* watchdog_get_task_name(WDG_TaskID_t task_id);
const char* watchdog_get_health_string(WDG_HealthStatus_t status);
```
- **功能**: 获取任务名称和状态的字符串描述

---

## 看门狗任务实现

### StartWatchdogTask

看门狗任务是一个独立的FreeRTOS任务，负责：
1. 每1秒检查所有任务的心跳状态
2. 每1秒喂一次IWDG
3. 每30秒打印一次统计信息
4. 检测到任务超时时记录日志

**任务优先级**: `osPriorityNormal`
**任务周期**: 1秒
**堆栈大小**: 256×4 = 1KB

**任务流程**:
```
启动 → 等待5秒系统稳定 → 主循环:
    1. 自身心跳
    2. 检查所有任务
    3. 喂IWDG
    4. 统计打印（每30秒）
    5. 延迟1秒
```

---

## 集成到项目

### 1. 文件添加

**头文件**: `Core/Inc/watchdog.h`
**源文件**: `Core/Src/watchdog.c`

### 2. 系统初始化

在 [freertos.c](Core/Src/freertos.c) 的 `MX_FREERTOS_Init()` 中：

```c
/* 7. 初始化看门狗保护系统 */
if (watchdog_init() != 0) {
    LOG_ERROR("[Main] Watchdog init failed!");
    Error_Handler();
}
LOG_INFO("[Main] ✓ Watchdog protection system initialized");

/* 检查是否因看门狗重启 */
if (watchdog_check_reset_reason()) {
    LOG_ERROR("[Main] ⚠️  Last reset was caused by watchdog!");
}
```

### 3. 任务心跳集成

在所有关键任务的主循环中添加心跳：

**DefaultTask** (LwIP主任务):
```c
void StartDefaultTask(void *argument)
{
    for(;;)
    {
        /* 处理网络事件和串口命令 */
        // ...

        /* 看门狗心跳 */
        watchdog_feed_task(WDG_TASK_DEFAULT);

        osDelay(10);
    }
}
```

**TCPSendTask** (TCP发送):
```c
void TCPSendTask(void *argument)
{
    for(;;)
    {
        /* 发送TCP数据 */
        // ...

        /* 看门狗心跳 */
        watchdog_feed_task(WDG_TASK_TCP_SEND);

        osDelay(10);
    }
}
```

**FDCANRxTask** (CAN接收):
```c
void FDCANRxTask(void *argument)
{
    for(;;)
    {
        /* 等待接收信号量 */
        osSemaphoreAcquire(fdcan_rx_semaphore, 100);

        /* 处理接收数据 */
        // ...

        /* 看门狗心跳 */
        watchdog_feed_task(WDG_TASK_FDCAN_RX);
    }
}
```

**MotorControlLoopTask** (电机控制):
```c
void MotorControlLoopTask(void *argument)
{
    for(;;)
    {
        /* 等待定时器触发 */
        osSemaphoreAcquire(timer_sem, osWaitForever);

        /* 执行电机控制 */
        // ...

        osThreadYield();

        /* 看门狗心跳 */
        watchdog_feed_task(WDG_TASK_MOTOR_CTRL);
    }
}
```

### 4. WatchdogTask实现

在 [freertos.c](Core/Src/freertos.c) 中：

```c
void WatchdogTask(void *argument)
{
    /* 调用看门狗模块的任务入口函数 */
    StartWatchdogTask(argument);
}
```

---

## 使用场景

### 场景1: 系统正常运行

```
所有任务正常工作 → 每个任务定期上报心跳 → 看门狗监控正常 → IWDG被定期喂狗 → 系统稳定运行
```

**日志输出**（每30秒）:
```
[Watchdog] ==================== Statistics ====================
[Watchdog] Check Cycles: 300
[Watchdog] IWDG Feed Count: 300 (last: 1000 ms ago)
[Watchdog] Total Timeouts: 0
[Watchdog] --- Task Health Status ---
[Watchdog]   DefaultTask: OK (heartbeats=3000, timeouts=0, last=100 ms ago)
[Watchdog]   MotorControlLoop: OK (heartbeats=1000, timeouts=0, last=100 ms ago)
[Watchdog]   TCPSend: OK (heartbeats=30000, timeouts=0, last=10 ms ago)
[Watchdog]   FDCANRx: OK (heartbeats=5000, timeouts=0, last=50 ms ago)
[Watchdog]   Watchdog: OK (heartbeats=300, timeouts=0, last=1000 ms ago)
[Watchdog] ====================================================
```

### 场景2: 任务卡死检测

```
电机任务卡死 → 超过1.5秒无心跳 → 看门狗检测到WARNING → 超过3秒无心跳 → 看门狗检测到TIMEOUT → 记录错误日志 → 可选择重启任务或复位系统
```

**日志输出**:
```
[Watchdog] Task 'MotorControlLoop' heartbeat slow (1800 ms since last)
[Watchdog] ⚠️  Task 'MotorControlLoop' TIMEOUT! (no heartbeat for 3500 ms)
```

### 场景3: 系统死机复位

```
系统完全卡死 → 看门狗任务无法运行 → IWDG超过5秒未喂狗 → IWDG触发系统复位 → 系统重启 → watchdog_check_reset_reason()检测到看门狗复位 → 记录日志
```

**日志输出**（重启后）:
```
[Main] ⚠️  Last reset was caused by watchdog!
```

---

## 故障处理策略

### 1. 任务超时处理

在 [watchdog.c](Core/Src/watchdog.c):290 的 `watchdog_check_all_tasks()` 函数中，可以添加超时处理逻辑：

```c
/* 如果有任务超时，考虑是否需要采取行动 */
if (any_timeout) {
    /* 选项A: 仅记录日志（当前实现）*/
    // 已自动记录LOG_ERROR

    /* 选项B: 触发紧急停止 */
    // robot_state_emergency_stop();

    /* 选项C: 尝试重启超时任务 */
    // restart_task(task_id);

    /* 选项D: 停止喂IWDG，让系统复位 */
    // return;  // 不再喂IWDG
}
```

### 2. IWDG复位统计

可以在FLASH中记录看门狗复位次数：

```c
bool watchdog_check_reset_reason(void)
{
    bool is_watchdog_reset = false;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != RESET) {
        LOG_ERROR("[Watchdog] ⚠️  System was reset by IWDG!");
        is_watchdog_reset = true;

        /* 可选: 增加复位计数并保存到FLASH */
        // watchdog_reset_counter++;
        // flash_save_reset_counter(watchdog_reset_counter);
    }

    __HAL_RCC_CLEAR_RESET_FLAGS();
    return is_watchdog_reset;
}
```

---

## 性能影响

### CPU占用

- **看门狗任务**: 每秒运行一次，占用 < 0.1%
- **心跳上报**: 每个任务每次循环调用一次，占用 < 0.01%
- **总体影响**: 可忽略不计

### 内存占用

- **任务监控表**: 5个任务 × ~32字节 = 160字节
- **互斥锁**: 1个 × 40字节 = 40字节
- **任务堆栈**: 256×4 = 1KB
- **总计**: 约 **1.2KB**

---

## 调试技巧

### 1. 实时查看任务状态

在串口调试界面添加命令：

```c
/* 在StartDefaultTask中添加 */
if (uart_rx_char == 'w' || uart_rx_char == 'W') {
    watchdog_print_stats();
}
```

### 2. 模拟任务卡死

测试看门狗功能：

```c
/* 在MotorControlLoopTask中添加 */
if (test_watchdog_timeout) {
    LOG_WARN("[Test] Simulating task hang for 10 seconds...");
    osDelay(10000);  /* 模拟卡死10秒 */
}
```

预期结果：
- 3秒后看门狗检测到TIMEOUT
- 5秒后IWDG复位系统（如果已启动）

### 3. 禁用IWDG启动

在开发调试阶段，可以暂时不启动IWDG：

```c
void StartWatchdogTask(void *argument)
{
    /* 延迟5秒，等待其他任务启动 */
    osDelay(5000);

    /* 启动IWDG（可选，调试时可注释掉）*/
    // watchdog_start_iwdg();  // 注释此行即可禁用IWDG

    /* 主循环... */
}
```

这样只有软件任务监控生效，不会因为调试断点触发系统复位。

---

## 常见问题

### Q1: IWDG启动后，调试器断点会导致系统复位吗？

**A**: 是的。IWDG在调试模式下仍然运行，超过5秒未喂狗会触发复位。

**解决方案**:
- 调试时不启动IWDG（注释掉 `watchdog_start_iwdg()`）
- 使用WWDG替代（Window WatchDog可在调试时暂停）

### Q2: 任务长时间阻塞（如等待信号量）会触发超时吗？

**A**: 会的。如果任务在等待信号量时超过3秒，看门狗会检测到超时。

**解决方案**:
- 在等待前后都上报心跳
- 使用带超时的等待（如`osSemaphoreAcquire(sem, 100)`）
- 在等待循环中定期上报心跳

### Q3: 如何添加新任务到监控列表？

**步骤**:
1. 在 [watchdog.h](Core/Inc/watchdog.h):34 添加任务ID枚举
```c
typedef enum {
    WDG_TASK_DEFAULT = 0,
    WDG_TASK_MOTOR_CTRL,
    WDG_TASK_TCP_SEND,
    WDG_TASK_FDCAN_RX,
    WDG_TASK_WATCHDOG,
    WDG_TASK_NEW_TASK,  /* 新增任务 */

    WDG_TASK_MAX
} WDG_TaskID_t;
```

2. 在 [watchdog.c](Core/Src/watchdog.c):30 添加任务名称
```c
static const char *task_name_strings[WDG_TASK_MAX] = {
    "DefaultTask",
    "MotorControlLoop",
    "TCPSend",
    "FDCANRx",
    "Watchdog",
    "NewTask"  /* 新增任务名称 */
};
```

3. 在新任务中添加心跳上报
```c
void NewTask(void *argument)
{
    for(;;)
    {
        /* 任务逻辑 */
        // ...

        /* 上报心跳 */
        watchdog_feed_task(WDG_TASK_NEW_TASK);

        osDelay(100);
    }
}
```

---

## 总结

### 优点

✅ **双重保护**: 硬件IWDG + 软件任务监控
✅ **自动化**: 看门狗任务自动运行，无需手动干预
✅ **低开销**: CPU占用 < 0.1%, 内存占用 < 2KB
✅ **灵活控制**: 可动态启用/禁用任务监控
✅ **详细统计**: 提供丰富的诊断和统计信息
✅ **易于集成**: 只需在任务中添加一行心跳代码

### 适用场景

- 长时间无人值守运行
- 高可靠性要求的嵌入式系统
- 复杂多任务系统的故障诊断
- 系统死机自动恢复

### 注意事项

⚠️ IWDG启动后不可停止
⚠️ 调试时可能需要禁用IWDG
⚠️ 任务长时间阻塞会触发超时
⚠️ 不要在中断中调用看门狗API（除了喂IWDG）

---

**最后更新**: 2026-02-11
**测试状态**: 待验证
**相关文件**:
- [watchdog.h](Core/Inc/watchdog.h) - 看门狗头文件
- [watchdog.c](Core/Src/watchdog.c) - 看门狗实现
- [freertos.c](Core/Src/freertos.c) - 系统集成
