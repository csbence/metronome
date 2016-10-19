#!/usr/bin/env python3

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mongo_client import MetronomeMongoClient
import seaborn as sns
from pandas import DataFrame

__author__ = 'Bence Cserna'

small_durations = [10, 20, 50, 100]
algorithms = ["A_STAR", "LSS_LRTA_STAR", "MO_RTS"]
wall_path = '/input/vacuum/variants/wall-2/wall_'
cups_path = '/input/vacuum/variants/cups-2/cups_'
tiles_path = '/input/tiles/korf/4/all/'


def flatten(experiment):
    experiment_configuration = experiment.pop('experimentConfiguration')

    return {**experiment, **experiment_configuration}


def plot_experiments(db, algorithms, domain, instance, termination_type, lookahead_type = "DYNAMIC"):
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


def main():
    db = MetronomeMongoClient()

    # tips = sns.load_dataset("tips")/

    experiments = construct_data_frame(db)
    # print(experiments)
    plot_experiments(experiments)


if __name__ == "__main__":
    main()
