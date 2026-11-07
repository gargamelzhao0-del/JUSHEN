/**
 ******************************************************************************
 * @file    test_framework.c
 * @brief   单元测试和集成测试框架 - 工业级测试框架实现
 * @version V1.0
 * @note    工业级代码优化：系统化测试，确保代码质量
 ******************************************************************************
 */

#include "test_framework.h"
#include "cmsis_os.h"
#include "error_handling.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 * 私有变量
 * ============================================================================
 */

static TestStats_t test_stats = {0};
static bool test_framework_initialized = false;

/* ============================================================================
 * 私有函数原型
 * ============================================================================
 */

static const char* test_result_to_string(TestResult_t result);
static const char* test_level_to_string(TestLevel_t level);

/* ============================================================================
 * 公共函数实现
 * ============================================================================
 */

/**
 * @brief 初始化测试框架
 * @return 成功返回true，失败返回false
 */
bool TestFramework_Init(void)
{
    if (test_framework_initialized) {
        return true;
    }

    /* 重置测试统计 */
    TestFramework_ResetStats();

    test_framework_initialized = true;
    LOG_INFO("Test framework initialized");
    return true;
}

/**
 * @brief 运行测试套件
 * @param suite 测试套件
 * @return 测试套件的整体结果
 */
TestResult_t TestFramework_RunSuite(const TestSuite_t* suite)
{
    if (!test_framework_initialized || suite == NULL) {
        return TEST_RESULT_ERROR;
    }

    LOG_INFO("Running test suite: %s", suite->name);
    LOG_INFO("Description: %s", suite->description);
    LOG_INFO("Level: %s", test_level_to_string(suite->level));
    LOG_INFO("Total tests: %u", suite->test_count);
    LOG_INFO("========================================");

    uint32_t start_time = TestUtils_GetTimestamp();
    uint32_t passed = 0;
    uint32_t failed = 0;
    uint32_t skipped = 0;
    uint32_t errors = 0;

    for (uint32_t i = 0; i < suite->test_count; i++) {
        const TestCase_t* test_case = &suite->test_cases[i];
        
        LOG_INFO("Running test: %s", test_case->name);
        LOG_INFO("Description: %s", test_case->description);
        
        TestResult_t result = TestFramework_RunTestCase(test_case);
        
        switch (result) {
            case TEST_RESULT_PASS:
                passed++;
                LOG_INFO("Test %s: PASSED", test_case->name);
                break;
            case TEST_RESULT_FAIL:
                failed++;
                LOG_ERROR("Test %s: FAILED", test_case->name);
                break;
            case TEST_RESULT_SKIP:
                skipped++;
                LOG_INFO("Test %s: SKIPPED", test_case->name);
                break;
            case TEST_RESULT_ERROR:
                errors++;
                LOG_ERROR("Test %s: ERROR", test_case->name);
                break;
            default:
                break;
        }
        
        LOG_INFO("----------------------------------------");
    }

    uint32_t end_time = TestUtils_GetTimestamp();
    uint32_t duration = end_time - start_time;

    LOG_INFO("Test suite %s completed in %u ms", suite->name, duration);
    LOG_INFO("Results: %u passed, %u failed, %u skipped, %u errors", 
             passed, failed, skipped, errors);
    LOG_INFO("========================================");

    /* 更新全局统计 */
    test_stats.total_tests += suite->test_count;
    test_stats.passed_tests += passed;
    test_stats.failed_tests += failed;
    test_stats.skipped_tests += skipped;
    test_stats.error_tests += errors;
    test_stats.total_time_ms += duration;

    /* 确定套件整体结果 */
    if (errors > 0) {
        return TEST_RESULT_ERROR;
    } else if (failed > 0) {
        return TEST_RESULT_FAIL;
    } else if (skipped == suite->test_count) {
        return TEST_RESULT_SKIP;
    } else {
        return TEST_RESULT_PASS;
    }
}

/**
 * @brief 运行单个测试用例
 * @param test_case 测试用例
 * @return 测试结果
 */
TestResult_t TestFramework_RunTestCase(const TestCase_t* test_case)
{
    if (!test_framework_initialized || test_case == NULL || test_case->function == NULL) {
        return TEST_RESULT_ERROR;
    }

    uint32_t start_time = TestUtils_GetTimestamp();
    TestResult_t result = TEST_RESULT_ERROR;

    /* 运行测试函数 */
    result = test_case->function();

    uint32_t end_time = TestUtils_GetTimestamp();
    uint32_t duration = end_time - start_time;

    /* 检查超时 */
    if (test_case->timeout_ms > 0 && duration > test_case->timeout_ms) {
        LOG_ERROR("Test %s timed out after %u ms (limit: %u ms)", 
                 test_case->name, duration, test_case->timeout_ms);
        return TEST_RESULT_ERROR;
    }

    return result;
}

/**
 * @brief 获取测试统计信息
 * @param stats 测试统计信息指针
 * @return 成功返回true，失败返回false
 */
bool TestFramework_GetStats(TestStats_t* stats)
{
    if (!test_framework_initialized || stats == NULL) {
        return false;
    }

    memcpy(stats, &test_stats, sizeof(TestStats_t));
    return true;
}

/**
 * @brief 重置测试统计
 */
void TestFramework_ResetStats(void)
{
    memset(&test_stats, 0, sizeof(TestStats_t));
}

/**
 * @brief 打印测试报告
 */
void TestFramework_PrintReport(void)
{
    if (!test_framework_initialized) {
        LOG_ERROR("Test framework not initialized");
        return;
    }

    LOG_INFO("========================================");
    LOG_INFO("Test Framework Report");
    LOG_INFO("========================================");
    LOG_INFO("Total tests: %u", test_stats.total_tests);
    LOG_INFO("Passed: %u", test_stats.passed_tests);
    LOG_INFO("Failed: %u", test_stats.failed_tests);
    LOG_INFO("Skipped: %u", test_stats.skipped_tests);
    LOG_INFO("Errors: %u", test_stats.error_tests);
    LOG_INFO("Total time: %u ms", test_stats.total_time_ms);
    
    if (test_stats.total_tests > 0) {
        float pass_rate = (float)test_stats.passed_tests / test_stats.total_tests * 100.0f;
        LOG_INFO("Pass rate: %.2f%%", pass_rate);
    }
    
    LOG_INFO("========================================");
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================
 */

/**
 * @brief 测试结果转字符串
 * @param result 测试结果
 * @return 测试结果字符串
 */
static const char* test_result_to_string(TestResult_t result)
{
    switch (result) {
        case TEST_RESULT_PASS:
            return "PASS";
        case TEST_RESULT_FAIL:
            return "FAIL";
        case TEST_RESULT_SKIP:
            return "SKIP";
        case TEST_RESULT_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 测试级别转字符串
 * @param level 测试级别
 * @return 测试级别字符串
 */
static const char* test_level_to_string(TestLevel_t level)
{
    switch (level) {
        case TEST_LEVEL_UNIT:
            return "UNIT";
        case TEST_LEVEL_INTEGRATION:
            return "INTEGRATION";
        case TEST_LEVEL_SYSTEM:
            return "SYSTEM";
        case TEST_LEVEL_REGRESSION:
            return "REGRESSION";
        default:
            return "UNKNOWN";
    }
}

/* ============================================================================
 * 测试工具函数实现
 * ============================================================================
 */

/**
 * @brief 延时函数（用于测试）
 * @param ms 延时毫秒数
 */
void TestUtils_Delay(uint32_t ms)
{
    osDelay(ms);
}

/**
 * @brief 获取当前时间戳（用于测试计时）
 * @return 当前时间戳（毫秒）
 */
uint32_t TestUtils_GetTimestamp(void)
{
    return HAL_GetTick();
}

/**
 * @brief 模拟随机数据（用于测试）
 * @param min 最小值
 * @param max 最大值
 * @return 随机值
 */
int32_t TestUtils_Random(int32_t min, int32_t max)
{
    static uint32_t seed = 12345;
    seed = (seed * 1103515245 + 12345) % 0x7FFFFFFF;
    return min + (seed % (max - min + 1));
}