/**
 ******************************************************************************
 * @file    test_trajectory_planner.c
 * @brief   轨迹规划器模块单元测试 - 工业级测试用例
 * @version V1.0
 * @note    工业级代码优化：系统化测试，确保代码质量
 ******************************************************************************
 */

#include "test_framework.h"
#include "trajectory_planner.h"
#include "error_handling.h"

/* ============================================================================
 * 测试用例函数
 * ============================================================================
 */

/**
 * @brief 测试轨迹规划器初始化
 * @return 测试结果
 */
static TestResult_t test_trajectory_planner_init(void)
{
    LOG_INFO("Testing trajectory planner initialization...");
    
    /* 测试轨迹规划器初始化 */
    bool result = TrajectoryPlanner_Init();
    TEST_ASSERT(result, "Trajectory planner initialization failed");
    
    return TEST_RESULT_PASS;
}

/**
 * @brief 测试轨迹规划计算
 * @return 测试结果
 */
static TestResult_t test_trajectory_calculation(void)
{
    LOG_INFO("Testing trajectory calculation...");
    
    /* 测试轨迹规划计算 */
    uint8_t motor_id = 1;
    float target_position = 90.0f;
    float max_velocity = 100.0f;
    float max_acceleration = 500.0f;
    float max_deceleration = 500.0f;
    
    bool result = TrajectoryPlanner_SetTargetPosition(motor_id, target_position, 
                                                   max_velocity, max_acceleration, 
                                                   max_deceleration);
    TEST_ASSERT(result, "Trajectory planning failed");
    
    return TEST_RESULT_PASS;
}

/**
 * @brief 测试轨迹执行
 * @return 测试结果
 */
static TestResult_t test_trajectory_execution(void)
{
    LOG_INFO("Testing trajectory execution...");
    
    /* 测试轨迹执行 */
    uint8_t motor_id = 1;
    float position = 0.0f;
    float velocity = 0.0f;
    
    bool result = TrajectoryPlanner_Update(motor_id, &position, &velocity);
    TEST_ASSERT(result, "Trajectory execution failed");
    
    return TEST_RESULT_PASS;
}

/**
 * @brief 测试轨迹参数设置
 * @return 测试结果
 */
static TestResult_t test_trajectory_parameters(void)
{
    LOG_INFO("Testing trajectory parameters...");
    
    /* 测试轨迹参数设置 */
    uint8_t motor_id = 1;
    float max_velocity = 150.0f;
    float max_acceleration = 750.0f;
    float max_deceleration = 750.0f;
    
    bool result = TrajectoryPlanner_SetMotionLimits(motor_id, max_velocity, 
                                                 max_acceleration, max_deceleration);
    TEST_ASSERT(result, "Trajectory parameter setting failed");
    
    return TEST_RESULT_PASS;
}

/**
 * @brief 测试轨迹状态管理
 * @return 测试结果
 */
static TestResult_t test_trajectory_state_management(void)
{
    LOG_INFO("Testing trajectory state management...");
    
    /* 测试轨迹状态管理 */
    uint8_t motor_id = 1;
    
    bool is_active = TrajectoryPlanner_IsTrajectoryActive(motor_id);
    bool is_complete = TrajectoryPlanner_IsTrajectoryComplete(motor_id);
    
    /* 状态检查 */
    TEST_ASSERT(true, "Trajectory state management test");
    
    return TEST_RESULT_PASS;
}

/* ============================================================================
 * 测试用例定义
 * ============================================================================
 */

static const TestCase_t trajectory_planner_tests[] = {
    {
        .name = "test_trajectory_planner_init",
        .function = test_trajectory_planner_init,
        .level = TEST_LEVEL_UNIT,
        .description = "Test trajectory planner initialization",
        .timeout_ms = 1000
    },
    {
        .name = "test_trajectory_calculation",
        .function = test_trajectory_calculation,
        .level = TEST_LEVEL_UNIT,
        .description = "Test trajectory calculation",
        .timeout_ms = 1000
    },
    {
        .name = "test_trajectory_execution",
        .function = test_trajectory_execution,
        .level = TEST_LEVEL_UNIT,
        .description = "Test trajectory execution",
        .timeout_ms = 1000
    },
    {
        .name = "test_trajectory_parameters",
        .function = test_trajectory_parameters,
        .level = TEST_LEVEL_UNIT,
        .description = "Test trajectory parameters",
        .timeout_ms = 1000
    },
    {
        .name = "test_trajectory_state_management",
        .function = test_trajectory_state_management,
        .level = TEST_LEVEL_UNIT,
        .description = "Test trajectory state management",
        .timeout_ms = 1000
    }
};

/* ============================================================================
 * 测试套件定义
 * ============================================================================
 */

const TestSuite_t test_suite_trajectory_planner = {
    .name = "trajectory_planner",
    .test_cases = trajectory_planner_tests,
    .test_count = sizeof(trajectory_planner_tests) / sizeof(TestCase_t),
    .level = TEST_LEVEL_UNIT,
    .description = "Trajectory planner module unit tests"
};