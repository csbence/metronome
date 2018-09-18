#pragma once

#include <easylogging++.h>
#include <iostream>
#include <unordered_set>
#include <vector>

#ifdef STREAM_GRAPH
#include "httplib.h"
#endif

namespace metronome {
class Visualizer {
 public:
  void addNode(std::size_t nodeId,
               std::size_t x = 0,
               std::size_t y = 0,
               std::size_t z = 1,
               double size = 1,
               const std::string& label = "") {
#ifdef STREAM_GRAPH
    std::ostringstream commandBuilder;

    bool alreadyVisualized = nodeIds.find(nodeId) != edgeIds.end();
    nodeIds.insert(nodeId);

    double r = 0;
    double g = 0;
    double b = 0;
    
    std::size_t scale = 10;

    // clang-format off
      commandBuilder << "{\"" << (alreadyVisualized ? "cn" : "an")
                     << R"(":{")" << nodeId
                     << R"(":{"label":")"  << label << "\""
                     << R"(,"size":)"   << size
                     << R"(,"r":)"      << r
                     << R"(,"g":)"      << g
                     << R"(,"b":)"      << b
                     << R"(,"x":)"      << x * scale
                     << R"(,"y":)"      << y * scale
                     << R"(,"z":)"      << z * scale
                     << "}}}";
    // clang-format on

    commands.push_back(commandBuilder.str());
#endif
  }

  void addEdge(std::size_t edgeId,
               std::size_t sourceNodeId,
               std::size_t targetNodeId,
               std::string label = "",
               double weight = 1) {
#ifdef STREAM_GRAPH
    std::ostringstream commandBuilder;

    bool alreadyVisualized = edgeIds.find(edgeId) != edgeIds.end();
    edgeIds.insert(edgeId);

    // clang-format off
    commandBuilder << "{\"" << (alreadyVisualized ? "ce" : "ae")
                   << R"(":{")" << edgeId
                   << R"(":{"source":")"      << sourceNodeId
                   << R"(","target":")"      << targetNodeId
                   << R"(","label2":")"       << label
                   << R"(","directed":true)"
                   << R"( ,"weight":)"      << weight
                   << "}}}";
    // clang-format on

    commands.push_back(commandBuilder.str());
#endif
  }

  void post() {
#ifdef STREAM_GRAPH
    if (commands.empty()) {
      return;
    }

    std::ostringstream commandBuilder;

    for (const auto& command : commands) {
      commandBuilder << command << "\r\n";
    }

    commandBuilder << std::endl;
    std::string commandString = commandBuilder.str();

    LOG(INFO) << "Post commands: " << commands.size();
    commands.clear();

    httplib::Client cli("localhost", 8080);

    const char* workspaceTarget = "/workspace1?operation=updateGraph";
    auto res = cli.Post(workspaceTarget, commandString, "plain/text");

    //        std::cout << res->status << std::endl;
    //        std::cout << res->body << std::endl;
    //        std::cout << "Graph test end" << std::endl;
#endif
  }

 private:
#ifdef STREAM_GRAPH
  std::vector<std::string> commands;
  std::unordered_set<std::size_t> nodeIds;
  std::unordered_set<std::size_t> edgeIds;

#endif
};
}  // namespace metronome
