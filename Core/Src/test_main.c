/**
 ******************************************************************************
 * @file    test_main.c
 * @brief   主测试文件 - 运行所有测试套件
 * @version V1.0
 * @note    工业级代码优化：系统化测试，确保代码质量
 ******************************************************************************
 */

#include "test_framework.h"
#include "error_handling.h"
#include "log_system.h"

/* ============================================================================
 * 测试运行函数
 * ============================================================================
 */

/**
 * @brief 运行所有测试套件
 * @return 测试结果
 */
TestResult_t run_all_tests(void)
{
    LOG_INFO("========================================");
    LOG_INFO("Running All Test Suites");
    LOG_INFO("========================================");
    
    TestResult_t overall_result = TEST_RESULT_PASS;
    
    /* 运行电机控制测试套件 */
    TestResult_t motor_result = TestFramework_RunSuite(&test_suite_motor_control);
    if (motor_result != TEST_RESULT_PASS) {
        overall_result = motor_result;
    }
    
    /* 运行轨迹规划器测试套件 */
    TestResult_t trajectory_result = TestFramework_RunSuite(&test_suite_trajectory_planner);
    if (trajectory_result != TEST_RESULT_PASS && overall_result == TEST_RESULT_PASS) {
        overall_result = trajectory_result;
    }
    
    /* 运行错误处理测试套件 */
    TestResult_t error_result = TestFramework_RunSuite(&test_suite_error_handling);
    if (error_result != TEST_RESULT_PASS && overall_result == TEST_RESULT_PASS) {
        overall_result = error_result;
    }
    
    LOG_INFO("========================================");
    LOG_INFO("All Test Suites Completed");
    LOG_INFO("========================================");
    
    /* 打印测试报告 */
    TestFramework_PrintReport();
    
    return overall_result;
}

/**
 * @brief 测试初始化函数
 * @return 成功返回true，失败返回false
 */
bool test_init(void)
{
    LOG_INFO("Initializing test framework...");
    
    /* 初始化日志系统 */
    if (!log_system_init()) {
        printf("Failed to initialize log system\r\n");
        return false;
    }
    
    /* 初始化错误处理系统 */
    if (!ErrorHandler_Init()) {
        LOG_ERROR("Failed to initialize error handling system");
        return false;
    }
    
    /* 初始化测试框架 */
    if (!TestFramework_Init()) {
        LOG_ERROR("Failed to initialize test framework");
        return false;
    }
    
    LOG_INFO("Test framework initialized successfully");
    return true;
}

/**
 * @brief 测试主函数
 * @return 测试结果
 */
int test_main(void)
{
    LOG_INFO("========================================");
    LOG_INFO("Test Framework Main");
    LOG_INFO("========================================");
    
    /* 初始化测试 */
    if (!test_init()) {
        LOG_ERROR("Test initialization failed");
        return 1;
    }
    
    /* 运行所有测试 */
    TestResult_t result = run_all_tests();
    
    LOG_INFO("========================================");
    LOG_INFO("Test Framework Complete");
    LOG_INFO("========================================");
    
    return (result == TEST_RESULT_PASS) ? 0 : 1;
}