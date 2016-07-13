#ifndef METRONOME_CONFIGURATIONEXECUTOR_HPP
#define METRONOME_CONFIGURATIONEXECUTOR_HPP

#include "Configuration.hpp"
#include "Result.hpp"
#include "OfflinePlanManager.hpp"
#include <algorithm/AStar.hpp>
#include <algorithm/LssLrtaStar.hpp>
#include <easylogging++.h>
#include <rapidjson/document.h>
#include <string>
namespace metronome {

static const std::string RAW_DOMAIN{"rawDomain"};
static const std::string DOMAIN_NAME{"domainName"};
static const std::string DOMAIN_PATH{"domainPath"};
static const std::string ALGORITHM_NAME{"algorithmName"};
static const std::string TERMINATION_CHECKER_TYPE{"terminationCheckerType"};
static const std::string ACTION_DURATION{"actionDuration"};
static const std::string TIME_LIMIT{"timeLimit"};

static const std::string DOMAIN_GRID_WORLD{"gridWorld"};

static const std::string ALGORITHM_A_STAR{"aStar"};
static const std::string ALGORITHM_LSS_LRTA_STAR{"lssLrtaStar"};

class ConfigurationExecutor {
public:
    Result executeConfiguration(Configuration configuration) {
        // todo validate configuration

        std::basic_istream domainInputStream{};

        if (configuration.HasMember(RAW_DOMAIN.c_str())) {
            // todo handle raw domain files
            std::string rawDomain{configuration[RAW_DOMAIN.c_str()].GetString()};
        } else if (configuration.HasMember(DOMAIN_PATH.c_str())) {
            std::string domainPath{configuration[DOMAIN_PATH.c_str()].GetString()};
            domainInputStream = std::ifstream{domainPath};
        } else {
            LOG(ERROR) << "Domain not found. Raw domain or domain path must be provided." << std::endl;
            // todo return result with error
        }

        std::string domainName{configuration[DOMAIN_NAME.c_str()].GetString()};

        if (domainName == DOMAIN_GRID_WORLD) {
            executeDomain<GridWorld>(configuration, std::move(domainInputStream));
        } else {
            LOG(ERROR) << "Unknown domain name: " << domainName << std::endl;
            // todo return result with error
        }
    }

private:
    template<typename Domain>
    Result executeDomain(const Configuration& configuration, std::basic_istream domainInputStream) {
        Domain domain{configuration, domainInputStream};

        if (!configuration.HasMember(ALGORITHM_NAME.c_str())) {
            LOG(ERROR) << "Algorithm name not found." << std::endl;
            // todo return result with error
        }

        std::string algorithName{configuration[ALGORITHM_NAME.c_str()].GetString()};

        if (algorithName == ALGORITHM_A_STAR) {
            executeOfflineAlgorithm<Domain, AStar>(configuration, domain);
        } else if (algorithName == ALGORITHM_LSS_LRTA_STAR) {
            executeRealTimeAlgorithm<Domain, LssLrtaStar>(configuration, domain);
        } else {
            LOG(ERROR) << "Unknown algorithm name: " << algorithName << std::endl;
            // todo return result with error
        }
    }

    template<typename Domain, typename Algorithm>
    void executeOfflineAlgorithm(const Configuration& configuration, const Domain& domain) {
        Algorithm algorithm{configuration};

        OfflinePlanManager<Domain, Algorithm> offlinePlanManager;
        const Result result = offlinePlanManager.plan(configuration, domain, std::move(algorithm));
    }

    template<typename Domain, typename Algorithm>
    void executeRealTimeAlgorithm(const Configuration& configuration, const Domain& domain) {
        Algorithm algorithm{configuration};

        RealTimePlanManager <Domain, Algorithm> offlinePlanManager;
        const Result result = offlinePlanManager.plan(configuration, domain, std::move(algorithm));
    }
};
}
#endif // METRONOME_CONFIGURATIONEXECUTOR_HPP
