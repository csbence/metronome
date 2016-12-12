#!/usr/bin/env python3

import json
import subprocess
import numpy as np
import copy
from subprocess import Popen, PIPE

import sys

from mongo_client import MetronomeMongoClient

__author__ = 'Bence Cserna'


def execute_metronome(executable, resources, configuration, timeout):
    nice = "nice -n 20"
    return execute_metronome_command(" ".join([nice, executable, resources]), configuration, timeout)


def execute_metronome_command(command, configuration, timeout):
    # proc = Popen(" ".join([nice, executable, resources]), stdin=PIPE, stdout=PIPE, stderr=PIPE, shell=True)

    json_configuration = json.dumps(configuration)

    try:
        print("Start to execute metronome.")
        result = subprocess.run(command, input=json_configuration.encode(),
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

        print("Failed: {} Succeeded: {}/{}".format(failed_count, succeeded_count, len(configurations)))

    successful = [x for x in gat if x != 0]
    failed_count = len(gat) - len(successful)
    succeeded_count = len(successful)

    print("Experiment completed!")
    print("Failed: {} Succeeded: {}".format(failed_count, succeeded_count))

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
                                       termination_type, action_durations, lookahead_type):
    # configuration = {
    #     "timeLimit": 150000000000,
    #     "domainInstanceName": "Manual test instance",
    #     "domainName": domain_type,
    #     "terminationType": termination_type
    # }

    configuration = {
        "timeLimit": 150000000000,
        "domainInstanceName": "Manual test instance",
        "actionDuration": 11,
        "firstIterationDuration": 15,
        "domainName": "GRID_WORLD",
        "terminationType": "EXPANSION",
        "commitmentType": "SINGLE",
        "octileMovement": False,
        "actionExecutionTime": 1
    }

    configurations = [configuration]
    configurations = cartesian_product(configurations, "algorithmName", algorithms)
    configurations = cartesian_product(configurations, "domainPath", domains)
    # configurations = cartesian_product(configurations, "actionDuration", action_durations)
    configurations = cartesian_product(configurations, "lookaheadType", lookahead_type)

    return configurations


def save_to_file(results, file_name):
    with open(file_name, 'w') as outfile:
        json.dump(results, outfile)


def save_to_db(results):
    db = MetronomeMongoClient()
    db.upload_results(results)


def main():
    print("Metronome python.")

    # domains = ["/input/tiles/korf/4/all/{}".format(x) for x in range(1, 100)]
    domains = ["/input/vacuum/bence/highways/highway_{}.vw".format(x) for x in range(10, 1001, 10)]
    # domains.extend(["/input/vacuum/variants/wall-2/wall_{}.vw".format(x) for x in range(0, 100)])
    # domains.extend(["/input/vacuum/variants/uniform-2/uniform_{}.vw".format(x) for x in range(0, 100)])

    configurations = generate_experiment_configurations(["LSS_LRTA_STAR", "SLOW_RTS"],
                                                        "GRID_WORLD", domains, "EXPANSION", [], ["DYNAMIC"])

    # configurations = generate_experiment_configurations(["LSS_LRTA_STAR", "MO_RTS"], "SLIDING_TILE_PUZZLE", 
    # domains, "EXPANSION", [100, 1000, 10000, 100000])

    results = run_experiments(list(reversed(configurations)))
    print("Execution completed")

    data = None
    # db = MetronomeMongoClient()
    # db.upload_results(results)
    #    data = db.get_results("A_STAR",
    #                          "GRID_WORLD",
    #                          "/input/vacuum/variants/cups-2/cups_",
    #                          "EXPANSION",
    #                          100)

    #    for value in data:
    #        print(value)
    print("Done")


if __name__ == "__main__":
    main()
