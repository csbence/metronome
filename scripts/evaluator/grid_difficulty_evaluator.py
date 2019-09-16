#! /usr/bin/env python3

import os
import sys
import argparse
import random
import copy
from subprocess import run, PIPE, Popen
import json
from shutil import copyfile
from Metronome import distributed_execution
from collections import defaultdict

__author__ = 'Bence Cserna'


def generate_filter_configs(domains):
    config_list = []

    for domain in domains:
        config = dict()
        config['algorithmName'] = 'TIME_BOUNDED_A_STAR'
        config['actionDuration'] = 1000000000
        config['domainName'] = 'GRID_WORLD'
        config['terminationType'] = 'TIME'
        config['lookaheadType'] = 'DYNAMIC'
        config['commitmentStrategy'] = 'SINGLE'
        config['heuristicMultiplier'] = 1.0
        config['weight'] = 1.0
        # remove leading . char in a relative path
        config['domainPath'] = domain
        config['algorithmLabel'] = '*'
        config_list.append(config)
        
        for w in [1.5, 2]:
            config2 = config.copy()
            config2['weight'] = w
            config2['actionDuration'] = 100000
            config2['algorithmLabel'] = f'w:{w}'
            config_list.append(config2)

        greedy_config = config.copy()
        greedy_config['actionDuration'] = 100000
        greedy_config['tbaStrategy'] = 'GBFS'
        greedy_config['algorithmLabel'] = 'w:G'

        cluster_config = config.copy()
        cluster_config['algorithmName'] = 'CLUSTER_RTS'
        cluster_config['clusterDepthLimit'] = 500
        cluster_config['actionDuration'] = 100000
        cluster_config['extractionCacheSize'] = 100
        cluster_config['clusterGrowth'] = True
        cluster_config['clusterGrowthRate'] = 1.001
        cluster_config['clusterNodeLimit'] = 10000000
        cluster_config['algorithmLabel'] = 'C'

        config_list.append(greedy_config)
        config_list.append(cluster_config)

    return config_list


def main(args):
    this_cwd = os.getcwd()

    if args.verbose:
        print(f'Metronome path: {args.filter}')

    # generated_domains = [f"input/vacuum/uniform40/1k8k/uniform1000_8000-{i}" for
    generated_domains = [
        f"../gridworld/tunnels40/1k10k/filtered/tunnels1000_10000" \
                             f"-{i}.vw" for i in range(0, 10)]

    configs = generate_filter_configs(generated_domains)

    sys.path.append(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

    if args.verbose:
        print('Begin filtering of generated domains')

    os.chdir('../..')
    results = distributed_execution(configs)
    os.chdir(this_cwd)

    ordered_results = defaultdict(list)
    for result in results:
        ordered_results[result['configuration']['domainPath']].append(result)

    for path, results_for_path in ordered_results.items():
        solution_costs = {}
        for result in results_for_path:
            solution_cost = result['pathLength']
            solution_costs[result['configuration']['algorithmLabel']] = \
                solution_cost

        instance_name = path.split('/')[-1]
        print(f"{instance_name}: {solution_costs}")


if __name__ == '__main__':
    # Begin Argument Definition

    parser = argparse.ArgumentParser()

    parser.add_argument('-p', '--path',
                        help='directory path to save the worlds. MUST BE RELATIVE PATH to cwd. That is a known issue, but no time to fix',
                        default='./gridworld')
    parser.add_argument('-v', '--verbose', help='increase output verbosity',
                        action='store_true')

    main(args=parser.parse_args())
