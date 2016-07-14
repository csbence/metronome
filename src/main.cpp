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
                       "\"domainInstanceName\" : \"input/vacuum/h_400.vw\",\n"
                       "\"actionDuration\" : 6000000,\n"
                       "\"domainName\" : \"GRID_WORLD\",\n"
                       "\"terminationType\" : \"time\",\n"
                       "\"list\" : [1, 2],\n"
                       "\"objects\" : {\"a\": [], \"b\": {}},\n"
                       "\"algorithmName\" : \"A_STAR\"}";

    //    const char* json = " { \"hello\" : \"world\", \"t\" : true , \"f\" : false, \"n\": null, \"i\":123, \"pi\": 3"
    //        ".1416, \"a\":[1, 2, 3, 4],  \"list\" : [1, 2]} ";
    printf("Original JSON:\n %s\n", json);

    rapidjson::Document document;
    document.Parse(json);

    //    rapidjson::Value& a = document["list"];   // This time we uses non-const reference.
    //    rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
    //    for (int i = 5; i <= 10; i++)
    //        a.PushBack(i, allocator);   // May look a bit strange, allocator is needed for potentially realloc. We
    //        normally uses the document's.

    // Fluent API
    //    a.PushBack("Lua", allocator).PushBack("Mio", allocator);

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);

    ConfigurationExecutor::executeConfiguration(Configuration());

    LOG(INFO) << "Json test: " << buffer.GetString() << std::endl;

    return 0;
}
