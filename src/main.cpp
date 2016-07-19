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
    LOG(INFO) << "Unstoppable precision!" << std::endl;

    const char* json = "{\"timeLimit\" : 150000000000,\n"
                       "\"domainPath\" : "
                       "\"/input/vacuum/dylan/uniform.vw\",\n"
                       "\"domainInstanceName\" : \"Manual test instance\",\n"
                       "\"actionDuration\" : 6000000,\n"
                       "\"domainName\" : \"GRID_WORLD\",\n"
                       "\"terminationType\" : \"time\",\n"
                       "\"list\" : [1, 2],\n"
                       "\"objects\" : {\"a\": [], \"b\": {}},\n"
                       "\"algorithmName\" : \"A_STAR\"}";

    printf("Original JSON:\n %s\n", json);

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
