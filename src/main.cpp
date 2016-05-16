#include "easylogging++.h"
#include "util/PriorityQueue.hpp"

INITIALIZE_EASYLOGGINGPP

int main() {
  LOG(INFO) << "Hello, World!" << std::endl;
  return 0;
}
