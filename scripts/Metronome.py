#!/usr/bin/env python3

import json
import subprocess
import numpy as np
from subprocess import Popen, PIPE

import sys


def execute_metronome(executable, resources, configuration, timeout):
    nice = "nice -n 20"
    # proc = Popen(" ".join([nice, executable, resources]), stdin=PIPE, stdout=PIPE, stderr=PIPE, shell=True)


    try:
        print("Start to execute metronome.")
        result = subprocess.run(" ".join([nice, executable, resources]), input=configuration.encode(),
                                timeout=timeout, check=True, stdout=PIPE, stderr=PIPE, shell=True)

        stdout, stderr = result.stdout, result.stderr
        print("Metronome output: ")
        print(result.stdout.decode())
        print(result.stderr.decode())
        print("Finish execution")
        sys.stdout.flush()
    except subprocess.TimeoutExpired as e:
        # stdout, stderr = e.stdout, result.stderr
        print("Experiment timed out")
        print(e.stdout.decode())
        print(e.stderr.decode())
        sys.stdout.flush()
        return 0
    except subprocess.CalledProcessError as e:
        print("Experiment failed!")
        print(e.stdout.decode())
        print(e.stderr.decode())
        sys.stdout.flush()
        return 0

    raw_result = stdout.decode().split("Result:", 2)[1]
    result = json.loads(raw_result)

    print("Parsed result: \n")
    print(result)

    sys.stdout.flush()
    return result["goalAchievementTime"]


def run_experiments():
    path = "../build/release/Metronome"
    resources = "../resources"
    configuration = """{{
  "timeLimit": 150000000000,
  "domainPath": "/input/tiles/korf/4/all/1",
  "domainInstanceName": "Manual test instance",
  "actionDuration": 50000,
  "domainName": "SLIDING_TILE_PUZZLE",
  "terminationType": "TIME",
  "algorithmName": "MO_RTS"
}}
"""
# "timeLimit": 150000000000,
# "domainPath": "/input/vacuum/dylan/cups.vw",
# "domainInstanceName": "Manual test instance",
# "actionDuration": 50000,
# "domainName": "GRID_WORLD",
# "terminationType": "TIME",
# "algorithmName": "LSS_LRTA_STAR"
    gat = []
    for i in range(1, 10):
        gat.append(execute_metronome(path, resources, configuration.format(i), timeout=60))

        successful = [x for x in gat if x != 0]
        failed_count = len(gat) - len(successful)
        succeeded_count = len(successful)

        print("Iteration: " + str(i))
        print("GATs: " + str(gat))
        print("Failed: {} Succeeded: {}".format(failed_count, succeeded_count))
        print("Avg of successful:{}".format(np.mean(successful) / 1000000000))

    successful = [x for x in gat if x != 0]
    failed_count = len(gat) - len(successful)
    succeeded_count = len(successful)

    print("Experiment completed!")
    print("GATs: " + str(gat))
    print("Failed: {} Succeeded: {}".format(failed_count, succeeded_count))
    print("Avg of successful:{}".format(np.mean(successful) / 1000000000))


def main():
    print("Metronome python.")

    run_experiments()

    print("Done")


if __name__ == "__main__":
    main()
