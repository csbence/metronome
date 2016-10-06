#!/usr/bin/env python3

import json
import subprocess
import numpy as np
import copy
import mongo_client
from subprocess import Popen, PIPE

import sys

__author__ = 'Bence Cserna'

def execute_metronome(executable, resources, configuration, timeout):
    nice = "nice -n 20"
    # proc = Popen(" ".join([nice, executable, resources]), stdin=PIPE, stdout=PIPE, stderr=PIPE, shell=True)

    json_configuration = json.dumps(configuration)

    try:
        print("Start to execute metronome.")
        result = subprocess.run(" ".join([nice, executable, resources]), input=json_configuration.encode(),
                                timeout=timeout, check=True, stdout=PIPE, stderr=PIPE, shell=True)

        stdout, stderr = result.stdout, result.stderr
        print("Metronome output: ")
        sys.stdout.flush()
        print(result.stdout.decode())
        print(result.stderr.decode())
        print("Finish execution")
        sys.stdout.flush()
    except subprocess.TimeoutExpired as e:
        # stdout, stderr = e.stdout, result.stderr
        print("Experiment timed out")
        sys.stdout.flush()
        print(e.stdout.decode())
        print(e.stderr.decode())
        sys.stdout.flush()
        return 0
    except subprocess.CalledProcessError as e:
        print("Experiment failed!")
        sys.stdout.flush()
        print(e.stdout.decode())
        print(e.stderr.decode())
        sys.stdout.flush()
        return 0

    raw_result = stdout.decode().split("Result:", 2)[1]
    result = json.loads(raw_result)

#    print("Parsed result: \n")
#    print(result)

    sys.stdout.flush()
    return result


def run_experiments(configurations):
    path = "../build/release/Metronome"
    resources = "../resources"
    gat = []

    results = []

    for configuration in configurations:
        result = execute_metronome(path, resources, configuration, timeout=60)
        result["experimentConfiguration"] = configuration
        results.append(result)

        gat.append(result["goalAchievementTime"])

        successful = [x for x in gat if x != 0]
        failed_count = len(gat) - len(successful)
        succeeded_count = len(successful)

 #       print("GATs: " + str(gat))
        print("Failed: {} Succeeded: {}".format(failed_count, succeeded_count))
   #     print("Avg of successful:{}".format(np.mean(successful)))

    successful = [x for x in gat if x != 0]
    failed_count = len(gat) - len(successful)
    succeeded_count = len(successful)

    print("Experiment completed!")
#    pr#int("GATs: " + str(gat))
    print("Failed: {} Succeeded: {}".format(failed_count, succeeded_count))
#    p#rint("Avg of successful:{}".format(np.mean(successful)))

    return results


def cartesian_product(configurations, key, values):
    joined_configurations = []
    for configuration in configurations:
        for value in values:
            deepcopy = copy.deepcopy(configuration)
            deepcopy[key] = value
            joined_configurations.append(deepcopy)

    return joined_configurations


def generate_experiment_configurations(algorithms, domain_type, domains,
                                       termination_type, action_durations):
    configuration = {
        "timeLimit": 150000000000,
        "domainInstanceName": "Manual test instance",
        "domainName": domain_type,
        "terminationType": termination_type
    }

    configurations = [configuration]
    configurations = cartesian_product(configurations, "algorithmName", algorithms)
    configurations = cartesian_product(configurations, "domainPath", domains)
    configurations = cartesian_product(configurations, "actionDuration", action_durations)

    return configurations


def main():
    print("Metronome python.")
    print(sys.argv[1])

    domains = ["/input/tiles/korf/4/all/{}".format(x) for x in range(1, 100)]
#    domains = ["/input/vacuum/variants/cups-2/cups_{}.vw".format(x) for x in range(0, 100)]
#    domains.extend(["/input/vacuum/variants/wall-2/wall_{}.vw".format(x) for x in range(0, 100)])

    configurations = generate_experiment_configurations(["LSS_LRTA_STAR", "MO_RTS"], "SLIDING_TILE_PUZZLE", domains, "EXPANSION", [100, 1000, 10000, 100000])

    results = run_experiments(configurations)
    print("Execution completed")

 #   print(results)
    db = mongo_client.open_connection()

    mongo_client.upload_results(db, results)

    print("Done")


if __name__ == "__main__":
    main()
