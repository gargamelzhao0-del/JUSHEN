/**
 ******************************************************************************
 * @file    test_error_handling.c
 * @brief   错误处理系统单元测试 - 工业级测试用例
 * @version V1.0
 * @note    工业级代码优化：系统化测试，确保代码质量
 ******************************************************************************
 */

#include "test_framework.h"
#include "error_handling.h"

/* ============================================================================
 * 测试用例函数
 * ============================================================================
 */

/**
 * @brief 测试错误处理系统初始化
 * @return 测试结果
 */
static TestResult_t test_error_handling_init(void)
{
    LOG_INFO("Testing error handling system initialization...");
    
    /* 测试错误处理系统初始化 */
    bool result = ErrorHandler_Init();
    TEST_ASSERT(result, "Error handling system initialization failed");
    
    return TEST_RESULT_PASS;
}

/**
 * @brief 测试错误记录功能
 * @return 测试结果
 */
static TestResult_t test_error_recording(void)
{
    LOG_INFO("Testing error recording...");
    
    /* 测试错误记录 */
    uint32_t error_id = ERROR_INFO(MODULE_MAIN, 0x0001, "Test info error");
    TEST_ASSERT(error_id > 0, "Error recording failed");
    
    error_id = ERROR_WARN(MODULE_MOTOR_CONTROL, 0x0002, "Test warning error");
    TEST_ASSERT(error_id > 0, "Warning recording failed");
    
    error_id = ERROR_ERROR(MODULE_TRAJECTORY, 0x0003, "Test error");
    TEST_ASSERT(error_id > 0, "Error recording failed");
    
    return TEST_RESULT_PASS;
}

/**
 * @brief 测试错误解决功能
 * @return 测试结果
 */
static TestResult_t test_error_resolution(void)
{
    LOG_INFO("Testing error resolution...");
    
    /* 测试错误解决 */
    uint32_t error_id = ERROR_ERROR(MODULE_MAIN, 0x0004, "Test error for resolution");
    TEST_ASSERT(error_id > 0, "Error recording failed");
    
    bool result = ErrorHandler_ResolveError(error_id);
    TEST_ASSERT(result, "Error resolution failed");
    
    return TEST_RESULT_PASS;
}

/**
 * @brief 测试错误统计功能
 * @return 测试结果
 */
static TestResult_t test_error_stats(void)
{
    LOG_INFO("Testing error statistics...");
    
    /* 测试错误统计 */
    ErrorStats_t stats;
    bool result = ErrorHandler_GetStats(&stats);
    TEST_ASSERT(result, "Error statistics retrieval failed");
    
    /* 检查统计数据 */
    TEST_ASSERT(stats.total_errors > 0, "Error statistics not updated");
    
    return TEST_RESULT_PASS;
}

/**
 * @brief 测试系统健康检查
 * @return 测试结果
 */
static TestResult_t test_system_health_check(void)
{
    LOG_INFO("Testing system health check...");
    
    /* 测试系统健康检查 */
    bool health = ErrorHandler_CheckSystemHealth();
    /* 系统应该是健康的，因为我们只记录了非致命错误 */
    TEST_ASSERT(health, "System health check failed");
    
    return TEST_RESULT_PASS;
}

/* ============================================================================
 * 测试用例定义
 * ============================================================================
 */

static const TestCase_t error_handling_tests[] = {
    {
        .name = "test_error_handling_init",
        .function = test_error_handling_init,
        .level = TEST_LEVEL_UNIT,
        .description = "Test error handling system initialization",
        .timeout_ms = 1000
    },
    {
        .name = "test_error_recording",
        .function = test_error_recording,
        .level = TEST_LEVEL_UNIT,
        .description = "Test error recording functionality",
        .timeout_ms = 1000
    },
    {
        .name = "test_error_resolution",
        .function = test_error_resolution,
        .level = TEST_LEVEL_UNIT,
        .description = "Test error resolution functionality",
        .timeout_ms = 1000
    },
    {
        .name = "test_error_stats",
        .function = test_error_stats,
        .level = TEST_LEVEL_UNIT,
        .description = "Test error statistics functionality",
        .timeout_ms = 1000
    },
    {
        .name = "test_system_health_check",
        .function = test_system_health_check,
        .level = TEST_LEVEL_UNIT,
        .description = "Test system health check",
        .timeout_ms = 1000
    }
};

/* ============================================================================
 * 测试套件定义
 * ============================================================================
 */

const TestSuite_t test_suite_error_handling = {
    .name = "error_handling",
    .test_cases = error_handling_tests,
    .test_count = sizeof(error_handling_tests) / sizeof(TestCase_t),
    .level = TEST_LEVEL_UNIT,
    .description = "Error handling system unit tests"
};