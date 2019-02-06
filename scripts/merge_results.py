#!/usr/bin/env python3

import sys
import json
import gzip

__author__ = 'Bence Cserna'


def read_data(file_name):
    if file_name.endswith('.gz'):
        with gzip.open(file_name, "rb") as file:
            return json.loads(file.read().decode("utf-8"))

    with open(file_name) as file:
        return json.load(file)


def merge(results_paths):
    merged = []
    for results_path in results_paths:
        merged += read_data(results_path)

    print(json.dumps(merged))


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Please provide at least two result files to merge. ")
    else:
        merge(sys.argv[1:])
