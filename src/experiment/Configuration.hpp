#pragma once

#include "MetronomeException.hpp"
#include "rapidjson/document.h"
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

static const std::string DOMAIN_GRID_WORLD{"GRID_WORLD"};
static const std::string DOMAIN_TRAFFIC{"TRAFFIC"};
static const std::string DOMAIN_TILES{"SLIDING_TILE_PUZZLE"};

static const std::string ALGORITHM_A_STAR{"A_STAR"};
static const std::string ALGORITHM_LSS_LRTA_STAR{"LSS_LRTA_STAR"};
static const std::string ALGORITHM_CLUSTER_RTS{"CLUSTER_RTS"};

static const std::string TERMINATION_CHECKER_TIME{"TIME"};
static const std::string TERMINATION_CHECKER_EXPANSION{"EXPANSION"};

static const std::string LOOKAHEAD_STATIC{"STATIC"};
static const std::string LOOKAHEAD_DYNAMIC{"DYNAMIC"};

static const std::string COMMITMENT_SINGLE{"SINGLE"};
static const std::string COMMITMENT_MULTIPLE{"MULTIPLE"};

class Configuration {
 public:
  Configuration() : document{} {};
  Configuration(const Configuration&) = default;
    Configuration(Configuration&&) = default;

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

  void checkKey(const std::string& key) const {
    if (!hasMember(key)) {
      throw metronome::MetronomeException("Invalid key: " + key);
    }
  }

  const rapidjson::Document& getJsonDocument() const { return document; }

 private:
  rapidjson::Document document;
};

}  // namespace metronome
