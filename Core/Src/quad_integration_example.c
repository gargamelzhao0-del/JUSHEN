/**
 ****************************************************************************************************
 * @file        quad_integration_example.c
 * @version     V1.0
 * @brief       四足机器狗电机控制系统 - 集成示例代码
 * @details     展示如何使用高层API进行电机控制
 ****************************************************************************************************
 * @attention
 *
 * 使用方法：
 * 1. 在freertos.c的StartDefaultTask或创建专门的控制任务中调用示例函数
 * 2. 根据实际需求修改电机ID映射和控制参数
 * 3. 确保TCP通信和足端力传感器已正确初始化
 *
 ****************************************************************************************************
 */

#include "quad_motor_api.h"
#include "motor_control_enhanced.h"
#include "tcp_motor_server.h"
#include "foot_force_sensor.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * 示例1：完整的系统初始化流程
 * ============================================================================ */

/**
 * @brief 完整的系统初始化示例
 * @note  在FreeRTOS启动后，主控制任务开始前调用
 */
void Example_SystemInit(void)
{
    printf("\n========== Quadruped Motor System Initialization ==========\n");

    /* 1. 初始化底层电机句柄系统 */
    printf("[1/5] Initializing motor handles...\n");
    if (motor_handles_init() != 0) {
        printf("[ERROR] Failed to initialize motor handles!\n");
        return;
    }

    /* 创建12个电机句柄（CAN1: 0-5, CAN2: 6-11）*/
    for (uint8_t i = 0; i < 12; i++) {
        uint8_t can_channel = (i < 6) ? 1 : 2;
        uint8_t device_id = i + 1;
        motor_handle_create(i, can_channel, device_id);
    }
    printf("    ✓ Created 12 motor handles\n");

    /* 2. 初始化足端力传感器 */
    printf("[2/5] Initializing foot force sensors...\n");
    if (foot_force_init() != 0) {
        printf("[ERROR] Failed to initialize foot force sensors!\n");
        return;
    }

    /* 设置滞后检测阈值（防止抖动）*/
    foot_force_set_contact_threshold_hysteresis(3.0f, 7.0f);  /* 低3N, 高7N */
    foot_force_set_support_threshold(20.0f);                  /* 支撑阈值20N */
    foot_force_set_debounce_count(3);                         /* 去抖动3次 */
    printf("    ✓ Foot sensors initialized with hysteresis\n");

    /* 3. 初始化高层四足控制API */
    printf("[3/5] Initializing quadruped motor API...\n");
    if (QuadMotor_Init() != 0) {
        printf("[ERROR] Failed to initialize quadruped motor API!\n");
        return;
    }
    printf("    ✓ Quadruped API initialized\n");

    /* 4. 配置控制参数 */
    printf("[4/5] Configuring control parameters...\n");

    /* 使能力位混合控制 */
    QuadMotor_EnableHybridControl(true);

    /* 配置摆动相参数（全局）*/
    SwingParams_t swing_params = {
        .torque_limit = 3.0f,
        .speed_limit = 120.0f,
        .accel_limit = 800.0f,
        .pos_kp = 1.5f
    };
    QuadMotor_ConfigSwingParams(&swing_params);

    /* 配置支撑相参数（为每个电机配置）*/
    for (uint8_t i = 0; i < 12; i++) {
        StanceParams_t stance_params = {
            .force_threshold = 5.0f,
            .torque_target = 8.0f,
            .pos_limit = 30.0f,
            .transition_time = 0.1f
        };
        QuadMotor_ConfigStanceParams(i, &stance_params);
    }
    printf("    ✓ Control parameters configured\n");

    /* 5. 启动系统 */
    printf("[5/5] Starting quadruped motor system...\n");
    if (QuadMotor_Start() != 0) {
        printf("[ERROR] Failed to start system!\n");
        return;
    }
    printf("    ✓ System started successfully!\n");

    printf("=========================================================\n\n");
}

/* ============================================================================
 * 示例2：从上位机接收数据并控制电机（核心示例）
 * ============================================================================ */

/**
 * @brief 主控制循环示例（50Hz）
 * @note  这是最核心的示例，展示如何使用一行代码控制12个电机
 */
void Example_MainControlLoop(void)
{
    printf("\n========== Main Control Loop Example ==========\n");
    printf("Running at 50Hz, press Ctrl+C to stop...\n\n");

    /* 模拟接收到的RL控制器数据（实际应从TCP接收）*/
    float target_angles[12];

    /* 控制循环 */
    for (uint32_t iteration = 0; iteration < 100; iteration++) {
        /* 1. 接收上位机RL控制器数据（TCP）*/
        /* 实际代码应该是：tcp_receive_rl_data(target_angles); */
        /* 这里使用模拟数据 */
        for (uint8_t i = 0; i < 12; i++) {
            target_angles[i] = 10.0f * sinf(iteration * 0.1f + i * 0.5f);
        }

        /* 2. ⭐一行代码搞定所有控制！⭐ */
        uint8_t result = QuadMotor_UpdateFromRL(target_angles);

        if (result != 0) {
            printf("[ERROR] Update failed at iteration %lu\n", iteration);
        }

        /* 3. 每10次迭代打印一次状态 */
        if (iteration % 10 == 0) {
            uint32_t total_updates, total_frames;
            float optimization_rate;
            QuadMotor_GetStats(&total_updates, &total_frames, &optimization_rate);

            printf("[Iter %3lu] Updates: %lu, CANFD Frames: %lu, Optimization: %.1f%%\n",
                   iteration, total_updates, total_frames, optimization_rate);

            /* 打印足端接触状态 */
            uint8_t contact_mask = foot_force_get_contact_mask();
            printf("            Foot Contact: FL=%d FR=%d RL=%d RR=%d\n",
                   (contact_mask & 0x01) ? 1 : 0,
                   (contact_mask & 0x02) ? 1 : 0,
                   (contact_mask & 0x04) ? 1 : 0,
                   (contact_mask & 0x08) ? 1 : 0);
        }

        /* 4. 50Hz控制频率（20ms周期）*/
        osDelay(20);
    }

    printf("\n========================================\n\n");
}

/* ============================================================================
 * 示例3：手动控制单条腿
 * ============================================================================ */

/**
 * @brief 手动控制单条腿示例
 */
void Example_ManualLegControl(void)
{
    printf("\n========== Manual Leg Control Example ==========\n");

    /* 设置前左腿为力位混合模式 */
    QuadMotor_SetLegMode(LEG_FRONT_LEFT, LEG_MODE_HYBRID);

    /* 设置前左腿的3个关节目标位置 */
    uint8_t front_left_motors[3] = {0, 1, 2};  /* 髋、大腿、小腿 */
    float target_positions[3] = {0.0f, 30.0f, -60.0f};

    uint8_t success = QuadMotor_SetBatchPositions(front_left_motors,
                                                   target_positions,
                                                   3);

    printf("Set %d motors successfully\n", success);

    /* 等待到达目标位置 */
    float tolerance = 2.0f;  /* 2度容差 */
    uint32_t timeout = 3000; /* 3秒超时 */
    uint32_t start_time = HAL_GetTick();

    while (!QuadMotor_IsLegReached(LEG_FRONT_LEFT, tolerance)) {
        if (HAL_GetTick() - start_time > timeout) {
            printf("[TIMEOUT] Front left leg did not reach target\n");
            break;
        }
        osDelay(10);
    }

    printf("Front left leg reached target position!\n");
    printf("============================================\n\n");
}

/* ============================================================================
 * 示例4：查询状态和诊断
 * ============================================================================ */

/**
 * @brief 系统状态查询和诊断示例
 */
void Example_StatusDiagnostics(void)
{
    printf("\n========== Status Diagnostics Example ==========\n");

    /* 1. 获取所有电机状态 */
    QuadMotorStatus_t motor_status[12];
    if (QuadMotor_GetAllStatus(motor_status) == 0) {
        printf("All motor status retrieved successfully\n");

        /* 打印前3个电机的状态 */
        for (uint8_t i = 0; i < 3; i++) {
            printf("  Motor[%d]: Pos=%.2f° Vel=%.1f Torq=%.2f Temp=%.1f°C\n",
                   i,
                   motor_status[i].position,
                   motor_status[i].velocity,
                   motor_status[i].torque,
                   motor_status[i].temperature);
        }
    }

    /* 2. 获取腿部状态 */
    QuadLegStatus_t leg_status[4];
    if (QuadMotor_GetLegStatus(leg_status) == 0) {
        printf("\nLeg Status:\n");
        for (uint8_t i = 0; i < 4; i++) {
            printf("  %s: Force=%.2fN Contact=%s Scene=%s\n",
                   quad_leg_name((QuadLegID_t)i),
                   leg_status[i].foot_force,
                   leg_status[i].is_contact ? "YES" : "NO",
                   quad_scene_name(leg_status[i].scene));
        }
    }

    /* 3. 检查错误 */
    if (QuadMotor_HasError()) {
        uint8_t error_motors[12];
        uint8_t error_count = QuadMotor_GetErrors(error_motors, 12);

        printf("\n[WARNING] %d motors have errors:\n", error_count);
        for (uint8_t i = 0; i < error_count; i++) {
            printf("  - Motor[%d]\n", error_motors[i]);
        }

        /* 清除错误 */
        QuadMotor_ClearErrors();
        printf("Errors cleared\n");
    } else {
        printf("\n✓ No errors detected\n");
    }

    /* 4. 打印详细系统状态 */
    printf("\n");
    QuadMotor_PrintStatus(true);  /* detailed=true */

    printf("=============================================\n\n");
}

/* ============================================================================
 * 示例5：紧急停止和安全控制
 * ============================================================================ */

/**
 * @brief 紧急停止示例
 */
void Example_EmergencyStop(void)
{
    printf("\n========== Emergency Stop Example ==========\n");

    /* 检测到紧急情况（例如：足端力过大）*/
    for (uint8_t i = 0; i < 4; i++) {
        float force = foot_force_get_filtered((QuadFootID_t)i);
        if (force > 150.0f) {  /* 过载阈值150N */
            printf("[EMERGENCY] Overload detected on foot %d: %.2fN\n", i, force);

            /* 立即紧急停止所有电机 */
            QuadMotor_EmergencyStop();

            printf("All motors emergency stopped!\n");
            break;
        }
    }

    printf("=========================================\n\n");
}

/* ============================================================================
 * 示例6：TCP通信集成示例
 * ============================================================================ */

/**
 * @brief TCP通信集成示例（与Jetson Orin NX通信）
 */
void Example_TCPIntegration(void)
{
    printf("\n========== TCP Communication Example ==========\n");

    /* 1. 连接到上位机（Jetson Orin NX）*/
    const char* jetson_ip = "192.168.1.20";
    uint16_t jetson_port = 8889;

    if (tcp_client_connect(jetson_ip, jetson_port)) {
        printf("✓ Connected to Jetson at %s:%d\n", jetson_ip, jetson_port);

        /* 2. 发送电机状态数据（可定期发送）*/
        QuadMotorStatus_t motor_status[12];
        QuadMotor_GetAllStatus(motor_status);

        /* 打包并发送（需要实现打包函数）*/
        // uint8_t packet_data[256];
        // Quadruped_PackStatusForTCP(motor_status, packet_data);
        // tcp_client_send_data(packet_data, 256);

        printf("Motor status sent to Jetson\n");

        /* 3. 接收RL控制器命令（在接收回调中处理）*/
        printf("Listening for RL controller commands...\n");

    } else {
        printf("[ERROR] Failed to connect to Jetson\n");
    }

    printf("=============================================\n\n");
}

/* ============================================================================
 * 主入口示例（在FreeRTOS任务中调用）
 * ============================================================================ */

/**
 * @brief 四足控制任务示例
 * @note  在freertos.c中创建此任务
 */
void QuadControlTask(void *argument)
{
    (void)argument;

    /* 等待系统初始化完成 */
    osDelay(1000);

    /* 执行系统初始化 */
    Example_SystemInit();

    /* 运行主控制循环 */
    while (1) {
        /* 选择合适的示例运行 */

        /* 示例1：TCP集成（实际应用）*/
        // Example_TCPIntegration();

        /* 示例2：主控制循环（核心）*/
        Example_MainControlLoop();

        /* 示例3：状态诊断 */
        Example_StatusDiagnostics();

        /* 延迟（根据需要调整）*/
        osDelay(5000);
    }
}
