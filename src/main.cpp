#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

int main() {
    LOG(INFO) << "Hello, World!" << std::endl;
    return 0;
}
