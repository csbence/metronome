#!/usr/bin/env python3

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mongo_client import MetronomeMongoClient
import seaborn as sns
from pandas import DataFrame
import json
import re

__author__ = 'Bence Cserna'

small_durations = [10, 20, 50, 100]
algorithms = ["A_STAR", "LSS_LRTA_STAR", "MO_RTS"]
wall_path = '/input/vacuum/variants/wall-2/wall_'
cups_path = '/input/vacuum/variants/cups-2/cups_'
tiles_path = '/input/tiles/korf/4/all/'


def flatten(experiment):
    experiment_configuration = experiment.pop('experimentConfiguration')

    return {**experiment, **experiment_configuration}


def plot_experiments(db, algorithms, domain, instance, termination_type, lookahead_type="DYNAMIC"):
    data = db.get_results(algorithms,
                          domain,
                          instance,
                          termination_type,
                          lookahead_type)

    if len(data) > 0:
        data_frame = construct_data_frame(data)
        plot_data_frame(data_frame)
    else:
        print("No results were found")


def construct_data_frame(data):
    flat_data = [flatten(experiment) for experiment in data]
    return DataFrame(flat_data)


def plot_data_frame(experiments):
    sns.set_style("white")

    boxplot = sns.boxplot(x="actionDuration",
                          y="goalAchievementTime",
                          hue="algorithmName",
                          data=experiments,
                          showmeans=True)
    plt.yscale('log')
    plt.show(boxplot)


def read_data(file_name):
    with open(file_name) as file:
        content = json.load(file)
    return content


def main():
    data = construct_data_frame(read_data("results_big2.txt"))
    data.drop(['errorMessage', 'commitmentType', "actionExecutionTime", "actionDuration", "success", "timeLimit",
               "terminationType", 'timestamp', 'octileMovement', 'lookaheadType', 'idlePlanningTime',
               'firstIterationDuration', 'generatedNodes', 'expandedNodes', 'domainInstanceName', 'domainName',
               'planningTime'],
              axis=1,
              inplace=True)

    data['domainPath'] = data['domainPath'].map(lambda x: int(re.findall(r'\d+', x)[0]))
    data.sort_values('domainPath', ascending=True, inplace=True)

    print(data)

    data = data.groupby('algorithmName').apply(lambda group, key: list(group[key]), 'goalAchievementTime')

    print(data)
    # data.plot()

    values = []
    for row in data.values:
        values.append([value for value in row])

    max_len = 0
    for row in values:
        max_len = max(len(row), max_len)

    for row in values:
        if len(row) < max_len:
            row += [float('NaN')] * (max_len - len(row))

    frame = DataFrame(np.asarray(values).T, columns=list(data.index))
    print(frame)
    # plt.figure()

    sns.set(font_scale=3, style='white')
    # sns.set_style("white")
    # x Instance size
    # y GAT
    # plt.figure(figsize=(6, 2))

    # mpl.rcParams.update({'font.size': 40})
    plot = frame.plot(linestyle='', marker='o', figsize=(15, 8))
    plot.set_xlabel('Instance size')
    plot.set_ylabel('Goal achievement time')
    plt.locator_params(axis='y', nbins=4)
    # plt.tight_layout()
    # plt.gcf().subplots_adjust(top=1.1)

    # plt.gcf().subplots_adjust(bottom=1)
    plt.tight_layout(pad=1, w_pad=0.5, h_pad=1.0)

    # plt.show()
    plt.savefig('highway_results.eps', format='eps')



    # db = MetronomeMongoClient()

    # tips = sns.load_dataset("tips")/
    # plot_experiments(db,
    #                  ["A_STAR", "LSS_LRTA_STAR", "MO_RTS", "MO_RTS_OLD"],
    #                  'SLIDING_TILE_PUZZLE',
    #                  '/input/tiles/korf/4/all/',
    #                  'EXPANSIONS',
    #                  'STATIC'
    #                  )

    # plot_experiments(db,
    #                  ["LSS_LRTA_STAR", "MO_RTS", "SLOW_RTS"],
    #                  'GRID_WORLD',
    #                  '/input/vacuum/variants/cups-2/cups_',
    #                  'EXPANSIONS',
    #                  'STATIC'
    #                  )


if __name__ == "__main__":
    main()
