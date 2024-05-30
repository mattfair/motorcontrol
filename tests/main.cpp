#include <CppUTest/CommandLineTestRunner.h>
#include "GTestOutput.h"

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
