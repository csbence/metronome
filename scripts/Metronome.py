#!/usr/bin/env python3

import json
import subprocess
import numpy as np
from subprocess import Popen, PIPE


def execute_metronome(executable, resources, configuration, timeout):
    nice = "nice -n 20"
    proc = Popen(" ".join([nice, executable, resources]), stdin=PIPE, stdout=PIPE, stderr=PIPE, shell=True)

    try:
        outs, errs = proc.communicate(input=configuration.encode(), timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        outs, errs = proc.communicate()
        print("Experiment timed out")
        return 0

    print("Metronome output: ")
    print(outs.decode())
    print(errs.decode())

    raw_result = outs.decode().split("Result:", 2)[1]
    result = json.loads(raw_result)

    print("Parsed result: \n")
    print(result)
    return result["goalAchievementTime"]


def run_experiments():
    path = "../build/release/Metronome"
    resources = "../resources"
    configuration = """{{
"timeLimit": 6000000,
"domainPath": "/input/tiles/korf/4/all/{}",
"domainInstanceName": "Manual test instance",
"actionDuration": 6000000,
"domainName": "SLIDING_TILE_PUZZLE",
"terminationType": "time",
"algorithmName": "LSS_LRTA_STAR"
}}
"""
    gat = []
    for i in range(1, 100):
        gat.append(execute_metronome(path, resources, configuration.format(i), timeout=60))

    successful = [x for x in gat if x != 0]
    failed_count = len(gat) - len(successful)
    succeeded_count = len(successful)

    print("GATs: " + str(gat))
    print("Failed: {} Succeeded: {}".format(failed_count, succeeded_count))
    print("Avg of successful:{}".format(np.mean(successful)))


def main():
    print("Metronome python.")

    run_experiments()

    print("Done")


if __name__ == "__main__":
    main()
