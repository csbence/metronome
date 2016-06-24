#ifndef VACUUM_WORLD_IO_HPP
#define VACUUM_WORLD_IO_HPP

#include <boost/assert.hpp>
#include <fstream>
#include <functional>
#include <vector>
#include <domains/VacuumWorld.hpp>
#include <iostream>

// template<typename T>
class VacuumWorldIO {
public:
  static VacuumWorld parseFromStream(const std::ifstream &inputFile) {

    VacuumWorld vacuumWorld;
    int rowCount = 0;
    int colCount = 0;

    try {
      /* TODO: read the inputFile */
      inputFile >> rowCount >> colCount;

    } catch (std::exception &e) {
      std::cout << e.what << '\n';
    }
    return vacuumWorld;
  }

private:
};

#endif
