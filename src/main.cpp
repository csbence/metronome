#include "easylogging++.h"
//#include "rapidjson/document.h"
//#include "rapidjson/stringbuffer.h"
//#include "rapidjson/writer.h"
#include <experiment/ConfigurationExecutor.hpp>

INITIALIZE_EASYLOGGINGPP

int main() {
    using namespace metronome;
    LOG(INFO) << "Unstoppable precision!" << std::endl;

    const char* json = "{\"timeLimit\" : 150000000000,\n"
                       "\"domainPath\" : \"input/vacuum/h_400.vw\",\n"
                       "\"domainInstanceName\" : "
                       "\"/Users/bencecserna/Documents/Development/projects/ai/metronome/resources/input/vacuum/dylan/"
                       "uniform.vw\",\n"
                       "\"actionDuration\" : 6000000,\n"
                       "\"domainName\" : \"GRID_WORLD\",\n"
                       "\"terminationType\" : \"time\",\n"
                       "\"list\" : [1, 2],\n"
                       "\"objects\" : {\"a\": [], \"b\": {}},\n"
                       "\"algorithmName\" : \"A_STAR\"}";

    printf("Original JSON:\n %s\n", json);

    //    rapidjson::Document document;
    //    document.Parse(json);
    //    rapidjson::StringBuffe   //    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    //    document.Accept(writer);

    const Result result = ConfigurationExecutor::executeConfiguration(Configuration(json));

    LOG(INFO) << "Execution completed in " << result.planningTime / 1000000 << "ms " << std::endl;
    LOG(INFO) << "Path length: " << result.pathLength << std::endl;

    //    LOG(INFO) << "Json test: " << buffer.GetString() << std::endl;

    return 0;
}
