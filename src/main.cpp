#include "easylogging++.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "util/TimeMeasurement.hpp"
#include <experiment/ConfigurationExecutor.hpp>

INITIALIZE_EASYLOGGINGPP

int main() {
    using namespace metronome;
    LOG(INFO) << "Unstoppable precision!" << std::endl;

    const char* json = "{\"timeLimit\" : 150000000000,\n"
                       "\"domainPath\" : \"input/vacuum/h_400.vw\",\n"
                       "\"domainInstanceName\" : \"/Users/bencecserna/Documents/Development/projects/ai/metronome/resources/input/vacuum/h_400.vw\",\n"
                       "\"actionDuration\" : 6000000,\n"
                       "\"domainName\" : \"GRID_WORLD\",\n"
                       "\"terminationType\" : \"time\",\n"
                       "\"list\" : [1, 2],\n"
                       "\"objects\" : {\"a\": [], \"b\": {}},\n"
                       "\"algorithmName\" : \"A_STAR\"}";

    printf("Original JSON:\n %s\n", json);

    rapidjson::Document document;
    document.Parse(json);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);

    ConfigurationExecutor::executeConfiguration(Configuration(json));

    LOG(INFO) << "Json test: " << buffer.GetString() << std::endl;

    return 0;
}
