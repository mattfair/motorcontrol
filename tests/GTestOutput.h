/**
 * This produces the output in the format of Google Test. Because I like the output of Google Test.
 */

#pragma once
#include <iostream>
#include <vector>
#include "CppUTest/SimpleString.h"
#include "CppUTest/TestFailure.h"
#include "CppUTest/TestOutput.h"
#include "CppUTest/TestRegistry.h"
#include "CppUTest/Utest.h"

// ANSI color codes
#define COLOR_RESET    "\033[0m"
#define COLOR_GREEN    "\033[32m"
#define COLOR_RED      "\033[31m"
#define COLOR_YELLOW   "\033[33m"

// convert SimpleString to std::string for ostream
std::ostream& operator<<(std::ostream& os, const SimpleString& str)
{
  return os << str.asCharString();
}

class GTestOutput : public TestOutput {
public:
    GTestOutput() : totalTests_(0), previousFailureCount_(0) {}

    void printTestsStarted() override {
        totalTests_ = static_cast<int>(TestRegistry::getCurrentRegistry()->countTests());
        std::cout << COLOR_GREEN "[==========]" COLOR_RESET " Running " << totalTests_ << " tests.\n";
    }

    void printTestsEnded(const TestResult& result) override {
        std::cout << COLOR_GREEN "[==========]" COLOR_RESET " " << result.getTestCount() << " tests ran.\n";
        std::cout << COLOR_GREEN "[  PASSED  ]" COLOR_RESET " " << result.getTestCount() - result.getFailureCount() << " tests.\n";
        if (!failures_.empty()) {
            std::cout << COLOR_RED "[  FAILED  ]" COLOR_RESET " " << result.getFailureCount() << " tests, listed below:\n";
            for (const auto& failure : failures_) {
                std::cout << COLOR_RED "[  FAILED  ]" COLOR_RESET " " << failure.groupName << "." << failure.testName << "\n";
            }
        }
    }

    void printCurrentTestStarted(const UtestShell& test) override {
        currentTestGroup_ = test.getGroup();
        currentTestName_ = test.getName();
        std::cout << COLOR_GREEN "[ RUN      ]" COLOR_RESET " " << currentTestGroup_ << "." << currentTestName_ << "\n";
    }

    void printCurrentTestEnded(const TestResult& result) override {
        if (result.getFailureCount() > previousFailureCount_) {
            std::cout << COLOR_RED "[  FAILED  ]" COLOR_RESET " " << currentTestGroup_ << "." << currentTestName_ << "\n";
            failures_.push_back({currentTestGroup_, currentTestName_});
        } else {
            std::cout << COLOR_GREEN "[       OK ]" COLOR_RESET " " << currentTestGroup_ << "." << currentTestName_ << "\n";
        }
        previousFailureCount_ = result.getFailureCount();
    }

    void printBuffer(const char* s) override {
        std::cout << s;
    }

    void flush() override {
        std::cout.flush();
    }

private:
    struct TestFailureInfo {
        SimpleString groupName;
        SimpleString testName;
    };

    std::vector<TestFailureInfo> failures_;
    int totalTests_;
    size_t previousFailureCount_;
    SimpleString currentTestGroup_;
    SimpleString currentTestName_;
};
