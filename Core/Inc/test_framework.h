/**
 ******************************************************************************
 * @file    test_framework.h
 * @brief   单元测试和集成测试框架 - 工业级测试框架
 * @version V1.0
 * @note    工业级代码优化：系统化测试，确保代码质量
 ******************************************************************************
 */

#ifndef __TEST_FRAMEWORK_H
#define __TEST_FRAMEWORK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "system_config.h"
#include "log_system.h"

/* ============================================================================
 * 测试类型定义
 * ============================================================================
 */

/**
 * @brief 测试结果枚举
 */
typedef enum {
    TEST_RESULT_PASS = 0,      /* 测试通过 */
    TEST_RESULT_FAIL = 1,      /* 测试失败 */
    TEST_RESULT_SKIP = 2,      /* 测试跳过 */
    TEST_RESULT_ERROR = 3      /* 测试错误 */
} TestResult_t;

/**
 * @brief 测试级别枚举
 */
typedef enum {
    TEST_LEVEL_UNIT = 0,       /* 单元测试 */
    TEST_LEVEL_INTEGRATION = 1, /* 集成测试 */
    TEST_LEVEL_SYSTEM = 2,     /* 系统测试 */
    TEST_LEVEL_REGRESSION = 3   /* 回归测试 */
} TestLevel_t;

/**
 * @brief 测试函数类型
 */
typedef TestResult_t (*TestFunction_t)(void);

/**
 * @brief 测试用例结构体
 */
typedef struct {
    const char* name;           /* 测试用例名称 */
    TestFunction_t function;    /* 测试函数 */
    TestLevel_t level;          /* 测试级别 */
    const char* description;    /* 测试描述 */
    uint32_t timeout_ms;        /* 测试超时时间 */
} TestCase_t;

/**
 * @brief 测试套件结构体
 */
typedef struct {
    const char* name;           /* 测试套件名称 */
    const TestCase_t* test_cases; /* 测试用例数组 */
    uint32_t test_count;        /* 测试用例数量 */
    TestLevel_t level;          /* 测试套件级别 */
    const char* description;    /* 测试套件描述 */
} TestSuite_t;

/**
 * @brief 测试统计信息
 */
typedef struct {
    uint32_t total_tests;       /* 总测试数 */
    uint32_t passed_tests;      /* 通过测试数 */
    uint32_t failed_tests;      /* 失败测试数 */
    uint32_t skipped_tests;     /* 跳过测试数 */
    uint32_t error_tests;       /* 错误测试数 */
    uint32_t total_time_ms;     /* 总测试时间 */
} TestStats_t;

/* ============================================================================
 * 测试框架API
 * ============================================================================
 */

/**
 * @brief 初始化测试框架
 * @return 成功返回true，失败返回false
 */
bool TestFramework_Init(void);

/**
 * @brief 运行测试套件
 * @param suite 测试套件
 * @return 测试套件的整体结果
 */
TestResult_t TestFramework_RunSuite(const TestSuite_t* suite);

/**
 * @brief 运行单个测试用例
 * @param test_case 测试用例
 * @return 测试结果
 */
TestResult_t TestFramework_RunTestCase(const TestCase_t* test_case);

/**
 * @brief 获取测试统计信息
 * @param stats 测试统计信息指针
 * @return 成功返回true，失败返回false
 */
bool TestFramework_GetStats(TestStats_t* stats);

/**
 * @brief 重置测试统计
 */
void TestFramework_ResetStats(void);

/**
 * @brief 打印测试报告
 */
void TestFramework_PrintReport(void);

/* ============================================================================
 * 测试断言宏
 * ============================================================================
 */

/**
 * @brief 断言测试条件为真
 */
#define TEST_ASSERT(condition, message) do { \
    if (!(condition)) { \
        LOG_ERROR("[TEST] Assertion failed: %s at %s:%d", message, __FILE__, __LINE__); \
        return TEST_RESULT_FAIL; \
    } \
} while(0)

/**
 * @brief 断言两个值相等
 */
#define TEST_ASSERT_EQ(expected, actual, message) do { \
    if ((expected) != (actual)) { \
        LOG_ERROR("[TEST] Assertion failed: %s - Expected: %d, Actual: %d at %s:%d", \
                 message, (int)(expected), (int)(actual), __FILE__, __LINE__); \
        return TEST_RESULT_FAIL; \
    } \
} while(0)

/**
 * @brief 断言两个浮点数相等（带误差范围）
 */
#define TEST_ASSERT_FLOAT_EQ(expected, actual, epsilon, message) do { \
    if (fabs((expected) - (actual)) > (epsilon)) { \
        LOG_ERROR("[TEST] Assertion failed: %s - Expected: %.6f, Actual: %.6f at %s:%d", \
                 message, (expected), (actual), __FILE__, __LINE__); \
        return TEST_RESULT_FAIL; \
    } \
} while(0)

/**
 * @brief 断言指针不为空
 */
#define TEST_ASSERT_NOT_NULL(ptr, message) do { \
    if ((ptr) == NULL) { \
        LOG_ERROR("[TEST] Assertion failed: %s - NULL pointer at %s:%d", message, __FILE__, __LINE__); \
        return TEST_RESULT_FAIL; \
    } \
} while(0)

/**
 * @brief 断言两个字符串相等
 */
#define TEST_ASSERT_STR_EQ(expected, actual, message) do { \
    if (strcmp((expected), (actual)) != 0) { \
        LOG_ERROR("[TEST] Assertion failed: %s - Expected: '%s', Actual: '%s' at %s:%d", \
                 message, (expected), (actual), __FILE__, __LINE__); \
        return TEST_RESULT_FAIL; \
    } \
} while(0)

/* ============================================================================
 * 测试套件定义
 * ============================================================================
 */

/* 外部测试套件声明 */
extern const TestSuite_t test_suite_motor_control;
extern const TestSuite_t test_suite_trajectory_planner;
extern const TestSuite_t test_suite_error_handling;
extern const TestSuite_t test_suite_security;
extern const TestSuite_t test_suite_system;

/* ============================================================================
 * 测试工具函数
 * ============================================================================
 */

/**
 * @brief 延时函数（用于测试）
 * @param ms 延时毫秒数
 */
void TestUtils_Delay(uint32_t ms);

/**
 * @brief 获取当前时间戳（用于测试计时）
 * @return 当前时间戳（毫秒）
 */
uint32_t TestUtils_GetTimestamp(void);

/**
 * @brief 模拟随机数据（用于测试）
 * @param min 最小值
 * @param max 最大值
 * @return 随机值
 */
int32_t TestUtils_Random(int32_t min, int32_t max);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_FRAMEWORK_H */