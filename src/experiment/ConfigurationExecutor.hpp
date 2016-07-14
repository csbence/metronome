#ifndef METRONOME_CONFIGURATIONEXECUTOR_HPP
#define METRONOME_CONFIGURATIONEXECUTOR_HPP

#include "Configuration.hpp"
#include "OfflinePlanManager.hpp"
#include "RealTimePlanManager.hpp"
#include "Result.hpp"
#include <algorithm/AStar.hpp>
#include <algorithm/LssLrtaStar.hpp>
#include <domains/GridWorld.hpp>
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
    static Result executeConfiguration(Configuration configuration) {
        // todo validate configuration

        std::string domainName{configuration.getString(DOMAIN_NAME)};

        if (domainName == DOMAIN_GRID_WORLD) {
            executeDomain<GridWorld>(configuration);
        } else {
            LOG(ERROR) << "Unknown domain name: " << domainName << std::endl;
            // todo return result with error
        }
    }

private:
    template <typename Domain>
    static Result executeDomain(const Configuration& configuration) {
        Domain domain{getDomain<Domain>(configuration)};

        if (!configuration.hasMember(ALGORITHM_NAME)) {
            LOG(ERROR) << "Algorithm name not found." << std::endl;
            // todo return result with error
        }

        std::string algorithmName{configuration.getString(ALGORITHM_NAME)};

        if (algorithmName == ALGORITHM_A_STAR) {
            executeOfflinePlanner<Domain, AStar<Domain>>(configuration, domain);
        } else if (algorithmName == ALGORITHM_LSS_LRTA_STAR) {
            executeRealTimePlanner<Domain, LssLrtaStar<Domain>>(configuration, domain);
        } else {
            LOG(ERROR) << "Unknown algorithm name: " << algorithmName << std::endl;
            // todo return result with error
        }
    }

    template <typename Domain>
    static Domain getDomain(const Configuration& configuration) {
        //        std::basic_istream<char> domainInputStream{};

        if (configuration.hasMember(RAW_DOMAIN)) {
            // todo handle raw domain files
            std::string rawDomain{configuration.getString(RAW_DOMAIN)};
        } else if (configuration.hasMember(DOMAIN_PATH)) {
            std::string domainPath{configuration.getString(DOMAIN_PATH)};
            std::fstream fileInputStream;

            fileInputStream.open(domainPath, std::fstream::in);
            Domain domain{configuration, fileInputStream};
            fileInputStream.close();

            return domain;

        } else {
            LOG(ERROR) << "Domain not found. Raw domain or domain path must be provided." << std::endl;
            // todo return result with error
        }
    }

    template <typename Domain, typename Planner>
    static void executeOfflinePlanner(const Configuration& configuration, const Domain& domain) {
        Planner planner{domain, configuration};

        OfflinePlanManager<Domain, Planner> offlinePlanManager;
        const Result result = offlinePlanManager.plan(configuration, domain, planner);
    }

    template <typename Domain, typename Planner>
    static void executeRealTimePlanner(const Configuration& configuration, const Domain& domain) {
        Planner planner{domain, configuration};

        RealTimePlanManager<Domain, Planner> offlinePlanManager;
        const Result result = offlinePlanManager.plan(configuration, domain, planner);
    }
};
}
#endif // METRONOME_CONFIGURATIONEXECUTOR_HPP
