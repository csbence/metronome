#pragma once

#include <sys/stat.h>
#include <string>

namespace metronome {

bool fileExists(const std::string& path) {
  struct stat buffer;
  return stat(path.c_str(), &buffer) != -1;
}

}  // namespace metronome
