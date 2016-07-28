#include "easylogging++.h"
//#include "rapidjson/document.h"
//#include "rapidjson/stringbuffer.h"
//#include "rapidjson/writer.h"
#include "rapidjson/filereadstream.h"
#include <experiment/ConfigurationExecutor.hpp>
//#include <iostream>
#include <cstdio>


INITIALIZE_EASYLOGGINGPP

void printSplashScreen();

int main(int argc, char** argv) {
    printSplashScreen();

    if (argc == 1) {
        std::cerr << "Resource path is not provided." << std::endl;
        return 1;
    }

    if (argc == 2) {
        std::cerr << "Configuration file path is not provided." << std::endl;
        return 1;
    }

    std::string resourceDir{argv[1]};
    std::string configurationPath{argv[2]};

    using namespace metronome;
    using namespace rapidjson;

    FILE* fp = fopen(configurationPath.c_str(), "r");
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document document;
    document.ParseStream(is);
    fclose(fp);

//    getchar();

    try {
        const Result result = ConfigurationExecutor::executeConfiguration(Configuration(std::move(document)),
                                                                          resourceDir);

        LOG(INFO) << "Execution completed in " << result.planningTime / 1000000 << "ms";
        LOG(INFO) << "Path length: " << result.pathLength;
        LOG(INFO) << "Nodes :: expanded: " << result.expandedNodes << " generated: " << result.generatedNodes;

        //        for (auto action : result.actions) {
        //            LOG(INFO) << action;
        //        }

    } catch (const MetronomeException& exception) {
        LOG(ERROR) << exception.what();
    }

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
