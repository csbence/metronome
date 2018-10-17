#ifdef _WIN32 //The windows section
#define NOMINMAX
#define _WINSOCKAPI_
//Compiler Error C4596 was causing compile errors in easylogging++.h
//From what I can find, seems to be a bug in VS. Recommendation is to turn it off :/
#pragma warning( disable : 4596 )
#include <winsock2.h>
#include <Windows.h>
#endif

#include "easylogging++.h"
#include "experiment/ConfigurationExecutor.hpp"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/File.hpp"

#include <cstdio>
#include <utils/Statistic.hpp>

//#define NDEBUG

INITIALIZE_EASYLOGGINGPP

void printSplashScreen();

int main(int argc, char** argv) {
  using namespace metronome;
  printSplashScreen();

  if (argc == 1) {
    std::cerr << "Invalid arguments. Please use one of the following ways to "
                 "invoke Metronome:\n"
              << "Metronome <resource path> <json configuration path>\n"
              << "OR\n"
              << "Metronome <resource path> \n"
                 "then provide the configuration using the standard input."
              << std::endl;

    return 1;
  }

  std::string resourceDir{argv[1]};

  rapidjson::Document document;

  if (argc == 2) {
    std::stringstream jsonStream;
    LOG(INFO) << "Parsing JSON configuration from stdin.";

    for (std::string line; std::getline(std::cin, line);) {
      if (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos) {
        break;  // Terminate paring on empty line
      }

      LOG(INFO) << line;

      jsonStream << line;
    }

    rapidjson::IStreamWrapper streamWrapper{jsonStream};
    document.ParseStream(streamWrapper);
  } else {
    LOG(INFO) << "Parsing JSON configuration file.";

    std::string configurationPath{argv[2]};

    if (!fileExists(configurationPath)) {
      std::cerr << "Invalid configuration file: " << configurationPath
                << std::endl;
      return 1;
    }

    std::ifstream configurationFile{configurationPath};
    rapidjson::IStreamWrapper streamWrapper{configurationFile};
    document.ParseStream(streamWrapper);
  }

  LOG(INFO) << "Configuration has been processed.";

  const Result result = ConfigurationExecutor::executeConfiguration(
      Configuration(std::move(document)), resourceDir);

  LOG(INFO) << "Execution completed in " << result.planningTime / 1000000
            << "ms";
  LOG(INFO) << "Path length: " << result.pathLength;
  LOG(INFO) << "Nodes :: expanded: " << result.expandedNodes
            << " generated: " << result.generatedNodes;

  //                for (auto action : result.actions) {
  //                    LOG(INFO) << action;
  //                }

  std::cout << "\n\nResult:\n#" << std::endl;
  std::cout << result.getJsonString();
  std::cout << std::flush;

  return 0;
}

void printSplashScreen() {
  std::cout << std::endl;
  std::cout << R"( ___            ___    )" << std::endl;
  std::cout << R"(|###\  ______  /###|   )" << std::endl;
  std::cout << R"(|#|\#\ \    / /#/|#|   )" << std::endl;
  std::cout << R"(|#| \#\ \  / /#/ |#|   )" << std::endl;
  std::cout << R"(|#|  \#\ \/ /#/  |#|   )" << std::endl;
  std::cout << R"(|#|      /\      |#|   )" << std::endl;
  std::cout << R"(|#|     /  \     |#|   )" << std::endl;
  std::cout << R"(|#|    /____\    |#|   )" << std::endl;
  std::cout << R"(---- Metronome  ----   )" << std::endl;
  std::cout << R"( When time matters!    )" << std::endl << std::endl;
}
