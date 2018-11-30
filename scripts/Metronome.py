#!/usr/bin/env python3

import sys
import copy
import json
import os
from subprocess import run, TimeoutExpired, PIPE
from tqdm import tqdm
from concurrent.futures import ThreadPoolExecutor
import pandas as pd
import itertools
import time
from distlre.distlre import DistLRE, Task, RemoteHost

__author__ = 'Bence Cserna, William Doyle, Kevin C. Gall'


def generate_base_configuration():
    # required configuration parameters
    algorithms_to_run = ['CLUSTER_RTS', 'TIME_BOUNDED_A_STAR']
    # algorithms_to_run = ['A_STAR']
    expansion_limit = [100000000]
    lookahead_type = ['DYNAMIC']
    time_limit = [300000000000]
    # action_durations = [1]  # Use this for A*
    # action_durations = [10000000, 12000000, 16000000, 20000000, 25000000, 32000000]
    action_durations = [50, 100, 250, 500, 1000]
    termination_types = ['EXPANSION']
    step_limits = [100000000]

    base_configuration = dict()
    base_configuration['algorithmName'] = algorithms_to_run
    # base_configuration['expansionLimit'] = expansion_limit
    base_configuration['lookaheadType'] = lookahead_type
    base_configuration['actionDuration'] = action_durations
    base_configuration['terminationType'] = termination_types
    # base_configuration['stepLimit'] = step_limits
    # base_configuration['timeLimit'] = time_limit
    base_configuration['commitmentStrategy'] = ['SINGLE']
    base_configuration['terminationTimeEpsilon'] = [5000000]  # 4ms

    compiled_configurations = [{}]

    for key, value in base_configuration.items():
        compiled_configurations = cartesian_product(compiled_configurations,
                                                    key, value)

    # Algorithm specific configurations
    weight = [3.0]
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'weight', weight,
                                                [['algorithmName',
                                                  'WEIGHTED_A_STAR']])

    # No configurable resource ratio for RES at this time
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'backlogRatio', [0.2],
                                                [['algorithmName',
                                                  'TIME_BOUNDED_A_STAR']])

    compiled_configurations = cartesian_product(compiled_configurations,
                                                'weight', [1.0, 1.4, 2.0,
                                                           2.8, 4.0, 8.0],
                                                [['algorithmName',
                                                  'TIME_BOUNDED_A_STAR']])

    compiled_configurations = cartesian_product(compiled_configurations,
                                                'clusterNodeLimit', [100000],
                                                [['algorithmName',
                                                  'CLUSTER_RTS']])

    compiled_configurations = cartesian_product(compiled_configurations,
                                                'clusterDepthLimit', [10,
                                                                      100,
                                                                      500,
                                                                      1000,
                                                                      10000],
                                                [['algorithmName',
                                                  'CLUSTER_RTS']])

    return compiled_configurations


def generate_tile_puzzle():
    configurations = generate_base_configuration()

    puzzles = []
    for puzzle in range(1, 11):
        puzzles.append(str(puzzle))

    puzzle_base_path = 'input/tiles/korf/4/real/'
    full_puzzle_paths = [puzzle_base_path + puzzle for puzzle in puzzles]

    configurations = cartesian_product(configurations, 'domainName',
                                       ['SLIDING_TILE_PUZZLE_4'])
    configurations = cartesian_product(configurations, 'domainPath',
                                       full_puzzle_paths)

    return configurations


def generate_grid_world():
    configurations = generate_base_configuration()

    domain_paths = []

    # Build all domain paths
    dao_base_path = 'input/vacuum/orz100d/orz100d.map_scen_'
    dao_paths = []
    minima1500_base_path = 'input/vacuum/minima1500/minima1500_1500-'
    minima1500_paths = []
    minima3000_base_path = 'input/vacuum/minima3k_300/minima3000_300-'
    minima3000_paths = []
    uniform1500_base_path = 'input/vacuum/uniform1500/uniform1500_1500-'
    uniform1500_paths = []
    for scenario_num in range(0, 10):  # large set 25
        n = str(scenario_num)
        dao_paths.append(dao_base_path + n)
        minima1500_paths.append(minima1500_base_path + n + '.vw')
        minima3000_paths.append(minima3000_base_path + n + '.vw')
        uniform1500_paths.append(uniform1500_base_path + n + '.vw')

    domain_paths.extend(dao_paths)
    domain_paths.extend(minima1500_paths)
    domain_paths.extend(
        minima3000_paths)  # this was not included in the large set
    domain_paths.extend(uniform1500_paths)

    configurations = cartesian_product(configurations, 'domainName',
                                       ['ORIENTATION_GRID'])
    configurations = cartesian_product(configurations, 'domainPath',
                                       domain_paths)

    return configurations


def cartesian_product(base, key, values, filters=None):
    new_base = []
    if filters is None:
        for item in base:
            for value in values:
                new_configuration = copy.deepcopy(item)
                new_configuration[key] = value
                new_base.append(new_configuration)
    else:
        for item in base:
            if all(filter_key in item and item[filter_key] == filter_value for
                   filter_key, filter_value in filters):
                new_base.extend(cartesian_product([item], key, values))
            else:
                new_base.append(item)

    return new_base


def distributed_execution(configurations):
    executor = create_local_distlre_executor()

    futures = []
    progress_bar = tqdm(total=len(configurations))
    tqdm.monitor_interval = 0

    for configuration in configurations:
        nice = "nice -n 20"
        executable = 'build/release/Metronome'
        resources = 'resources/'
        command = ' '.join([nice, executable, resources])
        json_configuration = f'{json.dumps(configuration)}\n'

        task = Task(command=command, meta=None, time_limit=30, memory_limit=10)
        task.input = json_configuration.encode()

        print(task.command)
        print(task.input)

        future = executor.submit(task)
        future.add_done_callback(lambda _: progress_bar.update())
        future.configuration = configuration

        futures.append(future)

    # start_experiment_notification(experiment_count=len(configurations))
    print('Experiments started')
    executor.execute_tasks()

    executor.wait()
    progress_bar.close()

    print('Experiments finished')
    # end_experiment_notification()

    results = construct_results(futures)

    return results


def construct_results(futures):
    results = []
    for future in futures:
        exception = future.exception()
        if exception:
            results.append({
                'configuration': future.configuration,
                'success': False,
                'errorMessage': 'exception ::' + str(exception)
            })
            continue

        result = future.result()
        # print(f'output: {result.output}')

        raw_output = result.output.splitlines()
        # print('Output:')
        # print('\n'.join(raw_output))
        # print('Error:')
        # print(result.error)
        if '#' not in raw_output:
            results.append({
                'configuration': future.configuration,
                'success': False,
                'errorMessage': 'exception :: output not found :: ' +
                                str(raw_output)
            })
            continue

        result_offset = raw_output.index('#') + 1

        if result_offset >= len(raw_output):
            results.append({
                'configuration': future.configuration,
                'success': False,
                'errorMessage': 'exception :: incomplete output :: ' +
                                str(raw_output)
            })
            continue

        output = json.loads(raw_output[result_offset])
        results.append(output)
    return results


def create_local_distlre_executor():
    executor = DistLRE(local_threads=4)

    return executor


def create_remote_distlre_executor():
    from slack_notification import start_experiment_notification, \
        end_experiment_notification

    import getpass
    HOSTS = ['ai' + str(i) + '.cs.unh.edu' for i in
             [1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15]]
    print('\nExecuting configurations on the following ai servers: ')
    print(HOSTS)

    # I would recommend setting up public key auth for your ssh
    # password = getpass.getpass("Password to connect to [ai.cs.unh.edu]")
    password = None
    remote_hosts = [RemoteHost(host, port=22, password=password) for host in
                    HOSTS]
    # Remove executor
    executor = DistLRE(remote_hosts=remote_hosts)

    return executor


def read_results_from_file(file_name):
    if file_name.endswith('.gz'):
        with gzip.open("input.json.gz", "rb") as file:
            return json.loads(file.read().decode("utf-8"))

    with open(file_name) as file:
        return json.load(file)


def inplace_merge_experiments(old_results, new_results):
    for new_result in new_results:
        replaced = False
        for i, old_result in enumerate(old_results):
            if old_result['configuration'] == new_result['configuration']:
                old_results[i] = new_result
                replaced = True
                break

        if not replaced:
            old_results.append(new_result)


def extract_configurations_from_failed_results(results):
    return [result['configuration'] for result in results if
            not result['success']]


def build_metronome():
    if not os.path.exists('build/release'):
        os.makedirs('build/release')

    os.chdir('build/release')

    return_code = run(
        ['cmake',
         '-DCMAKE_BUILD_TYPE=Release',
         '../..']).returncode

    if return_code != 0:
        os.chdir('../..')
        return False

    return_code = run(
        ['cmake --build . --target Metronome -- -j4'],
        # ['cmake --build . --target Metronome --clean-first -- -j4'],
        shell=True).returncode

    os.chdir('../..')
    return return_code == 0


def print_summary(results_json):
    results = pd.read_json(json.dumps(results_json))
    print('Successful: {}/{}'.format(results.success.sum(), len(results_json)))


def save_results(results_json, file_name):
    if not os.path.exists('results'):
        os.makedirs('results')

    with open(file_name, 'w') as outfile:
        json.dump(results_json, outfile)
    print(f'Results saved to {file_name}')


def label_algorithms(configurations):
    for configuration in configurations:
        if configuration['algorithmName'] == 'CLUSTER_RTS':
            configuration['algorithmLabel'] = configuration['algorithmName'] \
                                              + ' limit: ' \
                                              + str(configuration[
                                                        'clusterDepthLimit'])
        if configuration['algorithmName'] == 'TIME_BOUNDED_A_STAR':
            configuration['algorithmLabel'] = configuration['algorithmName'] \
                                              + ' weight: ' \
                                              + str(configuration[
                                                        'weight'])


def main():
    print(os.getcwd())
    os.chdir('..')

    recycle = False

    if not build_metronome():
        raise Exception(
            'Build failed.')
    print('Build complete!')

    file_name = 'results/results.json'

    if recycle:
        # Load previous configurations
        old_results = read_results_from_file(file_name)
        configurations = extract_configurations_from_failed_results(old_results)
    else:
        # Generate new domain configurations
        configurations = generate_grid_world()
        label_algorithms(configurations)
        # configurations = configurations[:10]  # debug - keep only one config

    print('{} configurations has been generated '.format(len(configurations)))

    start_time = time.perf_counter()
    results = distributed_execution(configurations)
    end_time = time.perf_counter()

    print(f"Experiment time: {end_time - start_time}s")

    if recycle:
        inplace_merge_experiments(old_results, results)
        results = old_results

    for result in results:
        result.pop('actions', None)
        result.pop('systemProperties', None)

    save_results(results, 'results/results_temp.json')

    save_results(results, file_name)
    print_summary(results)

    print('{} results has been received.'.format(len(results)))


if __name__ == '__main__':
    main()
