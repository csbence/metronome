#ifndef METRONOME_CONFIGURATIONEXECUTOR_HPP
#define METRONOME_CONFIGURATIONEXECUTOR_HPP

#include <easylogging++.h>
#include <rapidjson/document.h>
#include <MetronomeException.hpp>
#include <algorithms/AStar.hpp>
#include <algorithms/LssLrtaStar.hpp>
#include <domains/GridWorld.hpp>
#include <string>
#include "Configuration.hpp"
#include "OfflinePlanManager.hpp"
#include "RealTimePlanManager.hpp"
#include "Result.hpp"
namespace metronome {

class ConfigurationExecutor {
public:
    static Result executeConfiguration(const Configuration& configuration, const std::string& resourcesDir) {
        // todo validate configuration

        if (!configuration.hasMember(DOMAIN_NAME)) {
            LOG(ERROR) << "Domain name not found." << std::endl;
            return Result(configuration, "Missing: domainName");
        }

        std::string domainName{configuration.getString(DOMAIN_NAME)};

        if (domainName == DOMAIN_GRID_WORLD) {
            return executeDomain<GridWorld>(configuration, resourcesDir);
        } else {
            LOG(ERROR) << "Unknown domain name: " << domainName << std::endl;
            return Result(configuration, "Unknown: domainName");
        }
    }

    template <typename Domain>
    static Domain extractDomain(const Configuration& configuration, const std::string& resourcesDir) {
        return getDomain<Domain>(configuration,resourcesDir);
    }

private:
    template <typename Domain>
    static Result executeDomain(const Configuration& configuration, const std::string& resourcesDir) {
        if (!(configuration.hasMember(RAW_DOMAIN) || configuration.hasMember(DOMAIN_PATH))) {
            LOG(ERROR) << "Domain not found. Raw domain or domain path must be provided." << std::endl;
            return Result(configuration, "Missing: rawDomain AND domainPath");
        }

        Domain domain{getDomain<Domain>(configuration, resourcesDir)};

        if (!configuration.hasMember(ALGORITHM_NAME)) {
            LOG(ERROR) << "Algorithm name not found." << std::endl;
            return Result(configuration, "Missing: algorithmName");
        }

        std::string algorithmName{configuration.getString(ALGORITHM_NAME)};

        if (algorithmName == ALGORITHM_A_STAR) {
            return executeOfflinePlanner<Domain, AStar<Domain>>(configuration, domain);
        } else if (algorithmName == ALGORITHM_LSS_LRTA_STAR) {
            return executeRealTimePlanner<Domain, LssLrtaStar<Domain>>(configuration, domain);
        } else {
            LOG(ERROR) << "Unknown algorithms name: " << algorithmName << std::endl;
            return Result(configuration, "Unknown: algorithmName");
        }
    }

    template <typename Domain>
    static Domain getDomain(const Configuration& configuration, const std::string& resourcesDir) {
        if (configuration.hasMember(RAW_DOMAIN)) {
            std::string rawDomain{configuration.getString(RAW_DOMAIN)};
            std::stringstream stringStream{rawDomain};

            return Domain{configuration, stringStream};
        } else if (configuration.hasMember(DOMAIN_PATH)) {
            std::string domainPath{resourcesDir + configuration.getString(DOMAIN_PATH)};

            std::fstream fileInputStream;

            fileInputStream.open(domainPath, std::fstream::in);
            Domain domain{configuration, fileInputStream};
            fileInputStream.close();

            return domain;
        } else {
            throw MetronomeException("Domain not found.");
        }
    }

    template <typename Domain, typename Planner>
    static Result executeOfflinePlanner(const Configuration& configuration, const Domain& domain) {
        Planner planner{domain, configuration};

        OfflinePlanManager<Domain, Planner> offlinePlanManager;
        return offlinePlanManager.plan(configuration, domain, planner);
    }

    template <typename Domain, typename Planner>
    static Result executeRealTimePlanner(const Configuration& configuration, const Domain& domain) {
        Planner planner{domain, configuration};

        RealTimePlanManager<Domain, Planner> offlinePlanManager;
        return offlinePlanManager.plan(configuration, domain, planner);
    }
};
}
#endif // METRONOME_CONFIGURATIONEXECUTOR_HPP
