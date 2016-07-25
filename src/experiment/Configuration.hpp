#ifndef METRONOME_CONFIGURATION_HPP
#define METRONOME_CONFIGURATION_HPP

#include "rapidjson/document.h"
namespace metronome {

static const std::string RAW_DOMAIN{"rawDomain"};
static const std::string DOMAIN_NAME{"domainName"};
static const std::string DOMAIN_PATH{"domainPath"};
static const std::string DOMAIN_INSTANCE{"domainInstanceName"};
static const std::string ALGORITHM_NAME{"algorithmName"};
static const std::string TERMINATION_CHECKER_TYPE{"terminationCheckerType"};
static const std::string ACTION_DURATION{"actionDuration"};
static const std::string TIME_LIMIT{"timeLimit"};

static const std::string DOMAIN_GRID_WORLD{"GRID_WORLD"};

static const std::string ALGORITHM_A_STAR{"A_STAR"};
static const std::string ALGORITHM_VEHICLE{"VEHICLE"};
static const std::string ALGORITHM_LSS_LRTA_STAR{"LSS_LRTA_STAR"};
class Configuration {
public:
    Configuration() : document{} {};
    Configuration(const Configuration&) = default;
    Configuration(Configuration&&) = default;

    Configuration(rapidjson::Document document) : document{std::move(document)} {
    }

    Configuration(const std::string& json) : document{} {
        document.Parse(json.c_str());
    }

    bool hasMember(const std::string& key) const {
        return document.HasMember(key.c_str());
    }

    std::string getString(const std::string& key) const {
        return std::string{document[key.c_str()].GetString()};
    }

    long long int getLong(const std::string& key) const {
        return document[key.c_str()].GetInt64();
    }

private:
    rapidjson::Document document;
};
}

#endif // METRONOME_CONFIGURATION_HPP
