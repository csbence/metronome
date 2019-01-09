#pragma once

#include "MetronomeException.hpp"
#include "rapidjson/document.h"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <iostream>
#include <vector>

namespace metronome {

static const std::string RAW_DOMAIN{"rawDomain"};
static const std::string DOMAIN_NAME{"domainName"};
static const std::string DOMAIN_PATH{"domainPath"};
static const std::string DOMAIN_INSTANCE{"domainInstanceName"};
static const std::string ALGORITHM_NAME{"algorithmName"};
static const std::string TERMINATION_CHECKER_TYPE{"terminationType"};
static const std::string ACTION_DURATION{"actionDuration"};
static const std::string TIME_LIMIT{"timeLimit"};
static const std::string LOOKAHEAD_TYPE{"lookaheadType"};
static const std::string COMMITMENT_STRATEGY{"commitmentStrategy"};
static const std::string HEURISTIC_MULTIPLIER{"heuristicMultiplier"};

static const std::string DOMAIN_GRID_WORLD{"GRID_WORLD"};
static const std::string DOMAIN_ORIENTATION_GRID{"ORIENTATION_GRID"};
static const std::string DOMAIN_TRAFFIC{"TRAFFIC"};
static const std::string DOMAIN_TILES{"SLIDING_TILE_PUZZLE"};

static const std::string ALGORITHM_A_STAR{"A_STAR"};
static const std::string ALGORITHM_LSS_LRTA_STAR{"LSS_LRTA_STAR"};
static const std::string ALGORITHM_CLUSTER_RTS{"CLUSTER_RTS"};
static const std::string ALGORITHM_TIME_BOUNDED_A_STAR{"TIME_BOUNDED_A_STAR"};

static const std::string TERMINATION_CHECKER_TIME{"TIME"};
static const std::string TERMINATION_CHECKER_EXPANSION{"EXPANSION"};

static const std::string LOOKAHEAD_STATIC{"STATIC"};
static const std::string LOOKAHEAD_DYNAMIC{"DYNAMIC"};

static const std::string COMMITMENT_SINGLE{"SINGLE"};
static const std::string COMMITMENT_MULTIPLE{"MULTIPLE"};

// Cluster RTS
static const std::string CLUSTER_NODE_LIMIT{"clusterNodeLimit"};
static const std::string CLUSTER_DEPTH_LIMIT{"clusterDepthLimit"};
static const std::string EXTRACTION_CACHE_SIZE{"extractionCacheSize"};
static const std::string CLUSTER_WEIGHT("clusterWeight");

// TBA*
static const std::string PROJECTION{"projection"};

// Weighted Algorithms
static const std::string WEIGHT{"weight"};

class Configuration {
 public:
  Configuration() : document{} {};
  //  Configuration(const Configuration&) = default;
  Configuration(const Configuration&) : document{} {
    std::cout << "Here" << std::endl;
  };
  //    Configuration(Configuration&&) = default;
  Configuration(Configuration&&) : document{} {
    std::cout << "Here" << std::endl;
  };

  Configuration(rapidjson::Document document) : document{std::move(document)} {}

  Configuration(const std::string& json) : document{} {
    document.Parse(json.c_str());
  }

  bool hasMember(const std::string& key) const {
    return document.HasMember(key.c_str());
  }

  std::string getString(const std::string& key) const {
    checkKey(key);

    return std::string{document[key.c_str()].GetString()};
  }

  long long int getLong(const std::string& key) const {
    checkKey(key);

    return document[key.c_str()].GetInt64();
  }

  double getDouble(const std::string& key) const {
    checkKey(key);

    return document[key.c_str()].GetDouble();
  }

  double getBool(const std::string& key) const {
    checkKey(key);

    return document[key.c_str()].GetBool();
  }

  std::vector<double> getDoubles(const std::string& key) const {
    checkKey(key);

    auto genericArray = document[key.c_str()].GetArray();

    std::vector<double> doubles;
    doubles.reserve(genericArray.Size());

    for (const auto& value : genericArray) {
      doubles.push_back(value.GetDouble());
    }

    return doubles;
  }

  void checkKey(const std::string& key) const {
    if (!hasMember(key)) {
      throw metronome::MetronomeException("Missing configuration: " + key);
    }
  }

  const rapidjson::Document& getJsonDocument() const { return document; }

  friend std::ostream& operator<<(std::ostream& os,
                                  const Configuration& configuration) {
    using namespace rapidjson;

    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    configuration.getJsonDocument().Accept(writer);

    auto serializedConfiguration = std::string(buffer.GetString());
    os << "configuration: " << serializedConfiguration;

    return os;
  }

 private:
  rapidjson::Document document;
};

}  // namespace metronome
