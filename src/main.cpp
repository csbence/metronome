#include "easylogging++.h"
#include "rapidjson/document.h"
//#include "rapidjson/stringbuffer.h"
//#include "rapidjson/writer.h"
#include "experiment/ConfigurationExecutor.hpp"
#include "rapidjson/filereadstream.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/File.hpp"

//#include <iostream>
#include <cstdio>
#include <utils/statistic.hpp>
#define NDEBUG

INITIALIZE_EASYLOGGINGPP

void printSplashScreen();

int main(int argc, char** argv) {
    using namespace metronome;
    printSplashScreen();

    Statistic::initialize();

    if (argc == 1) {
        std::cerr << "Resource path is not provided. :: arg: " << argv[0] << std::endl;
        return 1;
    }

    std::string resourceDir{argv[1]};

    rapidjson::Document document;

    if (argc == 2) {
        std::stringstream jsonStream;

        for (std::string line; std::getline(std::cin, line);) {
            if (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos) {
                break; // Terminate paring on empty line
            }

            LOG(INFO) << line;

            jsonStream << line;
        }

        rapidjson::IStreamWrapper streamWrapper{jsonStream};
        document.ParseStream(streamWrapper);
    } else {
        std::string configurationPath{argv[2]};

        if (!fileExists(configurationPath)) {
            std::cerr << "Invalid configuration file: " << configurationPath << std::endl;
        }

        std::ifstream configurationFile{configurationPath};
        rapidjson::IStreamWrapper streamWrapper{configurationFile};
        document.ParseStream(streamWrapper);
    }

//        getchar(); // Wait for keypress

    const Result result = ConfigurationExecutor::executeConfiguration(Configuration(std::move(document)), resourceDir);

    LOG(INFO) << "Execution completed in " << result.planningTime / 1000000 << "ms";
    LOG(INFO) << "Path length: " << result.pathLength;
    LOG(INFO) << "Nodes :: expanded: " << result.expandedNodes << " generated: " << result.generatedNodes;

//                for (auto action : result.actions) {
//                    LOG(INFO) << action;
//                }

    std::cout << "\n\nResult:" << std::endl;
    std::cout << result.getJsonString();
    std::cout << std::flush;

    return 0;
}

void printSplashScreen() {
    std::cout << std::endl;
    std::cout << " ___            ___     " << std::endl;
    std::cout << "|###\\  ______  /###|   " << std::endl;
    std::cout << "|#|\\#\\ \\    / /#/|#|   " << std::endl;
    std::cout << "|#| \\#\\ \\  / /#/ |#|   " << std::endl;
    std::cout << "|#|  \\#\\ \\/ /#/  |#|   " << std::endl;
    std::cout << "|#|      /\\      |#|   " << std::endl;
    std::cout << "|#|     /  \\     |#|   " << std::endl;
    std::cout << "|#|    /____\\    |#|   " << std::endl;
    std::cout << "---- Metronome  ----" << std::endl;
    std::cout << " When time matters!" << std::endl << std::endl;
}
