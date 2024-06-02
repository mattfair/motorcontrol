//#include <CppUTest/CommandLineTestRunner.h>
//#include "GTestOutput.h"
#include "unity.h"

/*
class CustomCommandLineTestRunner : public CommandLineTestRunner {
public:
    CustomCommandLineTestRunner(int argc, const char* const* argv)
        : CommandLineTestRunner(argc, argv, TestRegistry::getCurrentRegistry()) {}

    TestOutput* createConsoleOutput() override {
        return new GTestOutput;
    }
};

int main(int argc, char** argv) {
    CustomCommandLineTestRunner runner(argc, argv);
    return runner.runAllTestsMain();
}
*/

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_function_should_doBlahAndBlah(void) {
    //test stuff
}

void test_function_should_doAlsoDoBlah(void) {
    //more test stuff
}

// not needed when using generate_test_runner.rb
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_function_should_doBlahAndBlah);
    RUN_TEST(test_function_should_doAlsoDoBlah);
    return UNITY_END();
}
