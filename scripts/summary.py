#!/usr/bin/env python3

import sys
import json
import pandas as pd
import gzip


def flatten(experiment):
    experiment_configuration = experiment.pop('configuration')
    return {**experiment, **experiment_configuration}


def construct_data_frame(data):
    flat_data = [flatten(experiment) for experiment in data]
    return pd.DataFrame(flat_data)


def read_data(file_name):
    if file_name.endswith('.gz'):
        with gzip.open(file_name, "rb") as file:
            return json.loads(file.read().decode("utf-8"))

    with open(file_name) as file:
        return json.load(file)


def summarize_result(result_path):
    result = construct_data_frame(read_data(result_path))

    print(f'Success rate: {result.success.sum()} / {len(result)}')
    print(f'Algorithms: {result.algorithmName.unique()}')
    print(f'Domains: {result.domainName.unique()}')
    print(f'Domains: {result.domainPath.unique()}')
    print('10 most frequent errors: ')
    print(result.errorMessage.value_counts()[:10])


def analyze(result_paths):
    for result_path in result_paths:
        print('#' * 80)
        print(f'Summary of {result_path}:')
        print('_' * 80)
        summarize_result(result_path)
        print('#' * 80)


if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Please provide result paths to summarize.")
    else:
        analyze(sys.argv[1:])
