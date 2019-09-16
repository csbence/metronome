#!/usr/bin/env python3

import gzip
import json
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import statsmodels.stats.api as sms
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.colors import ListedColormap
from pandas import DataFrame

__author__ = 'Bence Cserna, modified by Kevin C. Gall'

alg_map = {"A_STAR": "A*", "LSS_LRTA_STAR": "LSS-LRTA*", "SAFE_RTS": "SRTS",
           "S_ZERO": "S0", "SIMPLE_SAFE": "SS",
           "SINGLE_SAFE": "BEST_SAFE", "SAFE_RTS_TOP": "SRTS_TOP",
           "TIME_BOUNDED_A_STAR": "TBA*", "CES": "CES",
           "ENVELOPE": "Envelope v0.5", "ES": "RES", "CLUSTER_RTS":
               "Cluster-RTS"}


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
    # mpl.rcParams['axes.labelsize'] = 14
    # plt.tick_params(labelsize=14)
    # mpl.rcParams['xtick.top'] = True
    # mpl.rcParams['font.family'] = 'Serif'
    # mpl.rcParams['axes.labelsize']= 13

    mpl.rcParams['figure.subplot.left'] = 0.15
    font = {'family': 'Serif',
            # 'weight' : 'bold',
            'size': 13}

    mpl.rc('font', **font)


def add_row(df, values):
    return df.append(dict(zip(df.columns.values, values)), ignore_index=True)


def rename(row):
    # print(row)

    # if row.tbaStrategy == 'GBFS':
    #     row.algorithmLabel = 'TB-GBFS'

    if row.algorithmName == 'TIME_BOUNDED_A_STAR':
        if (row.weight != 1):
            row.algorithmLabel = 'TBwA* weight: ' + str(row.weight)

    return row


def plot_gat(data, plot_title, file_name):
    # print(f'Data to plot: {data}')
    print(data.algorithmLabel.unique())
    data.algorithmLabel = data.algorithmName.map(alg_map)

    print(data.algorithmLabel.unique())

    data = data.apply(rename, axis=1)

    print(data.algorithmLabel.unique())
    # data.algorithmLabel = data.algorithmName + ' ' + \
    # data.tbaStrategy.astype(str).replace(np.nan)
    # data.clusterGrowthRate.astype(str) + ' ' + \

    # data.algorithmName = data.algorithmLabel + data.clusterDepthLimit.astype(
    #     str)

    # results = DataFrame(
    #     columns="actionDuration withinOpt algorithmLabel lbound rbound".split())

    # rescale action durations to ms
    data['actionDuration'] = data['actionDuration'] / 1000000
    # data['goalAchievementTime'] = data['goalAchievementTime'] / 10e9

    # Change data structure such that goal achievement time is averaged,
    # grouped by action duration and algorithm
    # for fields, duration_group in data.groupby(
    #         ['algorithmLabel', 'actionDuration']):
    #     alg_name = fields[0]
    #     if alg_name in alg_map:
    #         alg_name = alg_map[alg_name]
    # 
    #     # Get mean of within optimal calculation, add row to results dataframe
    #     mean_within_opt = duration_group['withinOpt'].mean()
    #     within_opt_list = list(duration_group['withinOpt'])
    #     bound = sms.DescrStatsW(within_opt_list).zconfint_mean()
    #     results = add_row(results,
    #                       [fields[1], mean_within_opt, alg_name,
    #                        abs(mean_within_opt - bound[0]),
    #                        abs(mean_within_opt - bound[1])])

    # errors = []
    # for alg, alg_group in results.groupby('algorithmLabel'):
    #     errors.append([alg_group['lbound'].values, alg_group['rbound'].values])

    # pivot = results.pivot(index="actionDuration", columns="algorithmName",
    #                       values="withinOpt")
    # pivot = pivot[~pivot.index.duplicated(keep='first')]

    flatui = ["#9b59b6", "#3498db", "#95a5a6", "#e74c3c", "#34495e",
              "#2ecc71", "#f4df42"]

    flatui = sns.color_palette()

    color_palette = {
        'TBA*': flatui[4],
        'TBwA* weight: 1.5': flatui[1],
        'TBwA* weight: 2.0': flatui[3],
        'TBwA* weight: 2.5': flatui[6],
        'TBwA* weight: 8.0': flatui[8],
        'A*': flatui[7],
        'LSS-LRTA*': flatui[8],
        'TB-GBFS': flatui[5],
        'Cluster-RTS': flatui[0]
    }

    data.sort_values(['algorithmLabel'], inplace=True)
    print(data.columns)

    # data.backtrack = data.backtrack.fillna(0.1)
    # plot = sns.lineplot(x="actionDuration", y="backtrack",
    plot = sns.lineplot(x="actionDuration", y="pathLength",
                        # plot = sns.lineplot(x="actionDuration", y="withinOpt",
                        # plot = sns.lineplot(x="domainLabel", y="withinOpt",
                        hue="algorithmLabel", data=data,
                        # )
                        palette=color_palette)

    # plt.show()

    # return

    # Below is palette of distinguishable colors for analyzing large sets of algorithms together
    # colors = ["#90C3D4", "#C390D4", "#D4A190", "#A1D490", "#AB3299", "#AB8132", "#32AB44","#325DAB","#9BAB32", "#32AB7E","#4232AB","#AB325F","#495E49","#49545E","#5E495E", "#5E5449","#FA7887","#C8FA78","#78FAEB","#AA78FA"]
    # palette = sns.color_palette(n_colors=10)
    # plot = pivot.plot(color=palette, title=plot_title, legend=True, yerr=errors,
    #                   # ecolor='black',
    #                   elinewidth=0.5,
    #                   capsize=4, capthick=0.5)

    # plot.set_xscale('log')
    # plot.set_yscale('log')

    # plot.set_xticks([50, 100, 150, 250, 500, 1000, 2000, 3200])
    # plot.set_xticks([1, 3.2, 6.4, 12.8, 25.6])
    # plot.set_yticks([1, 2, 3])
    # plot.set_ylim([1.1, 2.8])
    # plot.set_xlim([1000, 10000])
    plot.get_xaxis().set_major_formatter(mpl.ticker.ScalarFormatter())
    plot.get_yaxis().set_major_formatter(mpl.ticker.ScalarFormatter())

    plot.set_xlabel('Planning Time per Iteration (milliseconds)')
    # plot.set_xlabel('Domain width')
    # plot.set_ylabel('Goal Achievement Time (Factor of Optimal)')
    # plot.set_ylabel('Number of backtracking actions')
    plot.legend(title="")

    handles, labels = plot.get_legend_handles_labels()
    plot.legend(handles=handles[1:], labels=labels[1:])

    pdf = PdfPages("../results/plots/" + file_name + ".pdf")
    plt.savefig(pdf, format='pdf')
    pdf.close()
    # plt.show()


def main(paths_to_base, paths, title, file_name, domain_token,
         expansion_delay):
    set_rc()

    data = prepare_data(paths, paths_to_base)
    data = filter_data(data, domain_token)

    # print(df)
    # df.plot(x='expansionDelay', y='withinOpt')
    # plt.show()

    plot_gat(data, title, file_name)


def filter_data(data, domain_token=None):
    if domain_token is not None:
        data = data[data['domainPath'].str.contains(domain_token)]
    print(f'size after domain name filtering: {len(data)}')

    # data = data[~data.algorithmLabel.str.contains('TIME_BOUNDED_A_STAR weight: 1.0')]

    # data = data[~(data.algorithmName == 'A_STAR')]
    data = data[(data.heuristicMultiplier == 1.0)]
    data.algorithmLabel = data.algorithmLabel + ' w: ' + data.weight.astype(str)
    # data = data[(data.domainName == 'ORIENTATION_GRID')]
    # data = data[data.algorithmLabel.str.contains('CLUSTER')]
    # data = data[data.algorithmLabel.str.contains(' cache: 100 ')]
    # data = data[~data.algorithmLabel.str.contains(' cache: 10 ')]
    # data = data[~data.algorithmLabel.str.contains(' 0.01 ')]
    # data = data[~data.algorithmLabel.str.contains(' cache: 10000 ')]
    # data = data[~data.algorithmLabel.str.contains('limit: 1000 ')]
    # data = data[~data.algorithmLabel.str.contains('limit: 10 ')]

    # non_zero_weight = ~data.algorithmLabel.str.contains('weight: 1.0')
    # gbfs = data.tbaStrategy.astype(str) == 'GBFS'
    # data = data[ non_zero_weight | gbfs ]
    # data = data[~gbfs]

    # data = data[data.clusterGrowthRate != 1.01]
    # data = data[data.weight <= 2.0]
    # data.loc[:,
    # 'algorithmName'] = data.algorithmName + '_' + data.weight.astype(
    #     str) + '_' + data.timeBoundedSearchStrategy.astype(str)
    # data.algorithmName.replace('[nan]+', '', inplace=True, regex=True)
    # df = data.groupby(['algorithmName', 'expansionDelay'], as_index=False).mean()
    return data


def prepare_data(paths, paths_to_base):
    results = []
    for path_name in paths:
        results += read_data(path_name)
    base_results = []
    if paths_to_base is not None:
        for base_path_name in paths_to_base:
            base_results += read_data(base_path_name)
    data = construct_data_frame(results + base_results)
    remove_unused_columns(data)
    action_durations = data.actionDuration.unique()
    print(f'Action duration used: {action_durations} \n'
          f'Original size with baseline: {len(data)}')

    #     data = data[~(data['errorMessage'].notnull() & (data.errorMessage !=
    #                                                     ""))]
    if (paths_to_base is not None):
        data = extrapolate_within_optimal(data)

    #
    astar = data[data["algorithmName"] == "A_STAR"]
    notastar = data[data["algorithmName"] != "A_STAR"]

    action_durations = notastar.actionDuration.unique()
    adf = DataFrame({'ad': action_durations})
    adf['actionDuration'] = 1
    cartesian = astar.merge(adf, how='outer')
    cartesian.goalAchievementTime = (cartesian.pathLength * cartesian.ad) + cartesian.planningTime
    cartesian.actionDuration = cartesian.ad

    print(len(action_durations))
    print(astar.shape)
    print(cartesian.shape)
    
    data = pd.concat([notastar, cartesian], ignore_index=True)


    print(f'Final size: {len(data)}')
    return data


def remove_unused_columns(data):
    data.drop(['actions', 'commitmentType',
               #                "success",
               "timeLimit",
               "terminationType", 'timestamp', 'octileMovement',
               'lookaheadType',
               'firstIterationDuration', 'generatedNodes', 'expandedNodes',
               "targetSelection", "safetyExplorationRatio", "safetyProof",
               "safetyWindowSize", "safetyBackup",
               'domainSeed', 'averageVelocity', "proofSuccessful", "rawDomain",
               "anytimeMaxCount",
               "systemProperties", "towardTopNode", "numberOfProofs",
               'terminationTimeEpsilon', 'backlogRatio', 'tbaOptimization'],
              axis=1,
              inplace=True,
              errors='ignore')


def extrapolate_within_optimal(data):
    astar = data[data["algorithmName"] == "A_STAR"]

    astar["optimalPathLength"] = astar["pathLength"]
    astar = astar[["domainPath", "optimalPathLength"]]

    data = pd.merge(data, astar, how='inner', on=["domainPath"])
    data["withinOpt"] = data["goalAchievementTime"] / (
            data["actionDuration"] * data["optimalPathLength"])

    check_opt_violation = False

    if check_opt_violation:
        violations = data[data.withinOpt < 1.0]
        if len(violations) > 0:
            print(violations)
            raise AssertionError(
                'Algorithms claim a better-than-optimal solution')

    return data


def extrapolate_a_star_results(data, action_durations):
    extrapolated_data = []

    for action_duration in action_durations:
        modified_data = data.copy()

        modified_data.actionDuration = action_duration
        modified_data.goalAchievementTime = modified_data.goalAchievementTime * action_duration + modified_data.planningTime

        extrapolated_data.append(modified_data)

    return pd.concat(extrapolated_data)


if __name__ == "__main__":
    # define command line usage
    # parser = argparse.ArgumentParser()
    # 
    # parser.add_argument("-b", "--paths_to_base", nargs="*",
    #                     help="Path to base results JSON")
    # parser.add_argument("-p", "--paths", nargs="*",
    #                     help="Path to experiment results JSON",
    #                     default=["../output/results.json"])
    # parser.add_argument("-i", "--individual",
    #                     help="Should plots be generated for each domain individually? (Primarily for debugging)",
    #                     action="store_true")
    # parser.add_argument("-t", "--title",
    #                     help="Title for plot (ignored for individual plots)",
    #                     default="Experiments")
    # parser.add_argument("-d", "--domain_token",
    #                     help="Domain token for filtering")
    # parser.add_argument("-e", "--expansion_delay",
    #                     help="Expansion delay for filtering")
    # 
    # args = parser.parse_args()

    paths_to_base = ['../results/c_grid_results_9+.json']
    paths_to_base = None
    paths = ['../results/c_grid_results_13.json',
             '../results/c_grid_results_14.json',
             # '../results/c_grid_results_9+.json',
             '../results/c_grid_results_15.json',
             '../results/c_grid_results_16.json',
             '../results/c_grid_results_18.json',
             # '../results/c_grid_results_17.json',
             ]

    # paths_to_base = ['../results/c_grid_results_19+.json']
    # paths = ['../results/c_grid_results_21.json',
    #          '../results/c_grid_results_20.json',
    #          ]

    # Backtrack
    # paths_to_base = None
    # paths = ['../results/c_grid_results_23.json',
    #          '../results/c_grid_results_24.json']

    # # Sliding tile
    # paths_to_base = ['../results/stp_baseline_results.json']
    # paths = ['../results/d_grid_results_5.json',
    #          '../results/d_grid_results_6.json',
    #          ]

    # # Size growth
    # paths_to_base = ['../results/f_grid_results_1.json']
    # paths = ['../results/f_grid_results_5.json',
    #          '../results/f_grid_results_6.json',
    #          ]

    # Size growth
    # paths_to_base = ['../results/f_grid_results_7.json']
    # paths = ['../results/f_grid_results_8.json',
    #          '../results/f_grid_results_9.json',
    #          '../results/f_grid_results_11.json',
    #          ]


    # Rebuttal LSS
    paths_to_base = None
    paths = ['../results/lss_grid_3k_M_STATIC.json',
             ]

    main(paths_to_base, paths, 'Dense Grid', 'lss_MS', None, None)

    # domain_name_lookup = {'aligned': 'Aligned Corridors',
    #                       'corridors': 'Corridors',
    #                       'minima': 'Minima 1500x1500',
    #                       'uniform': 'Uniform'}
    # expansion_lookup = {1000000: '1M', 100000: '100k', 20000: '20k', 1: '20000'}
    # for expansion_delay in [50000]:
    #     for domain_token in ['aligned', 'corridors', 'minima', 'uniform']:
    #         expansion_per_second = 1000000000 / expansion_delay
    #         domain_name = domain_name_lookup[domain_token]
    #         plot_name = 'domain instance: ' + domain_name + '         EPS: ' + str(
    #             expansion_lookup[expansion_per_second])
    #         file_name = domain_token + '_delay' + str(expansion_delay)
    # 
    #         main(paths_to_base, paths, plot_name, file_name,
    #              domain_token, expansion_delay)
