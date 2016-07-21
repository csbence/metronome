#include "easylogging++.h"
//#include "rapidjson/document.h"
//#include "rapidjson/stringbuffer.h"
//#include "rapidjson/writer.h"
#include <experiment/ConfigurationExecutor.hpp>

INITIALIZE_EASYLOGGINGPP

int main(int argc, char** argv) {
    if (argc == 1) {
        std::cerr << "Resource path is not provided." << std::endl;
        return 1;
    }

    std::string resourceDir{argv[1]};

    using namespace metronome;

    std::cout << std::endl;
    std::cout << " ___            ___     " << std::endl;
    std::cout << "|###\\  ______  /###|   " << std::endl;
    std::cout << "|#|\\#\\ \\    / /#/|#|   " << std::endl;
    std::cout << "|#| \\#\\ \\  / /#/ |#|   " << std::endl;
    std::cout << "|#|  \\#\\ \\/ /#/  |#|   " << std::endl;
    std::cout << "|#|      /\\      |#|   " << std::endl;
    std::cout << "|#|     /  \\     |#|   " << std::endl;
    std::cout << "|#|    /____\\    |#|   " << std::endl;
    std::cout << "---- Metronome  ----"<< std::endl;
    std::cout << " When time matters!" << std::endl << std::endl;

    const char* json = "{\"timeLimit\" : 150000000000,\n"
                       "\"domainPath\" : "
                       "\"/input/vacuum/dylan/uniform.vw\",\n"
                       "\"domainInstanceName\" : \"Manual test instance\",\n"
                       "\"actionDuration\" : 6000000,\n"
                       "\"domainName\" : \"GRID_WORLD\",\n"
                       "\"terminationType\" : \"time\",\n"
                       "\"list\" : [1, 2],\n"
                       "\"algorithmName\" : \"LSS_LRTA_STAR\"}";

//    printf("Original JSON:\n %s\n", json);

    try {
        const Result result = ConfigurationExecutor::executeConfiguration(Configuration(json), resourceDir);

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
