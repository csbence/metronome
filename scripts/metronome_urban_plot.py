#!/usr/bin/env python3

import gzip
import json
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import statsmodels.stats.api as sms
from matplotlib.backends.backend_pdf import PdfPages
from pandas import DataFrame

__author__ = 'Bence Cserna, modified by Kevin C. Gall'

alg_map = {"A_STAR": "A*", "LSS_LRTA_STAR": "LSS-LRTA*", "SAFE_RTS": "SRTS",
           "S_ZERO": "S0", "SIMPLE_SAFE": "SS",
           "SINGLE_SAFE": "BEST_SAFE", "SAFE_RTS_TOP": "SRTS_TOP",
           "TIME_BOUNDED_A_STAR": "TBA*", "CES": "CES",
           "ENVELOPE": "Envelope v0.5", "ES": "RES"}


def flatten(experiment):
    experiment_configuration = experiment.pop('configuration')
    return {**experiment, **experiment_configuration}


def construct_data_frame(data):
    flat_data = [flatten(experiment) for experiment in data]
    return DataFrame(flat_data)


def read_data(file_name):
    if file_name.endswith('.gz'):
        with gzip.open(file_name, "rb") as file:
            return json.loads(file.read().decode("utf-8"))

    with open(file_name) as file:
        return json.load(file)


def set_rc():
    mpl.rcParams['axes.labelsize'] = 10
    mpl.rcParams['xtick.top'] = True
    mpl.rcParams['font.family'] = 'Serif'


def add_row(df, values):
    return df.append(dict(zip(df.columns.values, values)), ignore_index=True)


def plot_gat(data, plot_title, file_name):
    print(f'Data to plot: {data}')
    data.algorithmName = data.algorithmLabel
    results = DataFrame(
        columns="actionDuration withinOpt algorithmName lbound rbound".split())

    # rescale action durations to ms
    data['actionDuration'] = data['actionDuration'] / 1000000

    # Change data structure such that goal achievement time is averaged,
    # grouped by action duration and algorithm
    for fields, duration_group in data.groupby(
            ['algorithmName', 'actionDuration']):
        alg_name = fields[0]
        if alg_name in alg_map:
            alg_name = alg_map[alg_name]

        # Get mean of within optimal calculation, add row to results dataframe
        mean_within_opt = duration_group['withinOpt'].mean()
        within_opt_list = list(duration_group['withinOpt'])
        bound = sms.DescrStatsW(within_opt_list).zconfint_mean()
        results = add_row(results,
                          [fields[1], mean_within_opt, alg_name,
                           abs(mean_within_opt - bound[0]),
                           abs(mean_within_opt - bound[1])])

    errors = []
    for alg, alg_group in results.groupby('algorithmName'):
        errors.append([alg_group['lbound'].values, alg_group['rbound'].values])

    pivot = results.pivot(index="actionDuration", columns="algorithmName",
                          values="withinOpt")
    pivot = pivot[~pivot.index.duplicated(keep='first')]

    palette = sns.color_palette(n_colors=10)
    plot = pivot.plot(color=palette, title=plot_title, legend=True, yerr=errors,
                      ecolor='black', elinewidth=1,
                      capsize=4, capthick=1)

    # plot.set_xscale('log')
    # plot.set_yscale('log')

    # plot.set_xticks([50, 100, 150, 250, 500, 1000, 2000, 3200])
    # plot.set_yticks([1, 1.1, 1.5, 2])
    # plot.set_ylim([1, 1.4])
    plot.get_xaxis().set_major_formatter(mpl.ticker.ScalarFormatter())

    plot.get_yaxis().set_major_formatter(mpl.ticker.ScalarFormatter())

    plot.set_xlabel('Planning Time per Iteration (milliseconds)')
    plot.set_ylabel('Goal Achievement Time (Factor of Optimal)')
    plot.legend(title="")

    pdf = PdfPages("../results/plots/" + file_name + ".pdf")
    plt.savefig(pdf, format='pdf')
    pdf.close()
    # plt.show()


def main(paths):
    set_rc()

    results = []
    for path_name in paths:
        results += read_data(path_name)

    data = construct_data_frame(results)
    remove_unused_columns(data)
    data.replace({'algorithmLabel': {
        'URBAN_A_STAR safety: 0': 'Plan to Stop',
        'URBAN_A_STAR safety: 2': 'SafeTPL',
    }}, inplace=True)

    data['Algorithm'] = data.algorithmLabel
    data['Planning Time [s]'] = data.planningTime
    data['Expanded Nodes'] = data.expandedNodes

    data['Average Velocity [m/s]'] = 100 / data.actionExecutionTime / 1.0e9
    
    # plot = sns.violinplot(data=data, x='Algorithm', y='Average Velocity ['
    #                                                   'm/s]')
    # 
    # 
    # file_name = 'velocity'
    # pdf = PdfPages("../results/plots/" + file_name + ".pdf")
    # plt.savefig(pdf, format='pdf')

    # plot = sns.boxplot(data=data, x='Algorithm', y='Planning Time [s]')

    # file_name = 'planningTime'
    # pdf = PdfPages("../results/plots/" + file_name + ".pdf")
    # plt.savefig(pdf, format='pdf')

    plot = sns.boxplot(data=data, x='Algorithm', y='Expanded Nodes')
    plot.set_yscale('log')

    file_name = 'expandedNodes'
    pdf = PdfPages("../results/plots/" + file_name + ".pdf")
    plt.savefig(pdf, format='pdf')


def remove_unused_columns(data):
    data.drop(['actions', 'commitmentType', "success", "timeLimit",
               "terminationType", 'timestamp', 'octileMovement',
               'lookaheadType',
               'firstIterationDuration',
               "targetSelection", "safetyExplorationRatio", "safetyProof",
               "safetyWindowSize", "safetyBackup",
               'domainSeed', 'averageVelocity', "proofSuccessful", "rawDomain",
               "anytimeMaxCount",
               "systemProperties", "towardTopNode", "numberOfProofs",
               'terminationTimeEpsilon', 'backlogRatio', 'tbaOptimization'],
              axis=1,
              inplace=True,
              errors='ignore')


if __name__ == "__main__":
    paths = ['../results/results.json']
    main(paths)
