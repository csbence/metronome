#!/usr/bin/env python

import json
import subprocess
from subprocess import Popen, PIPE


def execute_metronome(executable, resources, configuration, timeout):
    nice = "nice -n 20"
    proc = Popen(" ".join([nice, executable, resources]), stdin=PIPE, stdout=PIPE, stderr=PIPE, shell=True)

    try:
        outs, errs = proc.communicate(input=configuration.encode(), timeout=timeout)
    except TimeoutExpired:
        proc.kill()
        outs, errs = proc.communicate()

    print("Metronome output: ")
    print(outs.decode())
    print(errs.decode())

    raw_result = outs.decode().split("Result:", 2)[1]
    result = json.loads(raw_result)

    print("Parsed result: \n")
    print(result)


def main():
    print("Metronome python.")

    path = "../build/release/metronome"
    resources = "../resources"
    configuration = """{
"timeLimit": 150000000000,
"domainPath": "/input/tiles/korf/4/all/3",
"domainInstanceName": "Manual test instance",
"actionDuration": 6000000,
"domainName": "SLIDING_TILE_PUZZLE",
"terminationType": "time",
"algorithmName": "LSS_LRTA_STAR"
}
"""

    execute_metronome(path, resources, configuration, timeout=60)

    print("Done")


if __name__ == "__main__":
    main()
