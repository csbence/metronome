import math
import os
import re
import itertools

import matplotlib.cbook as cbook
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.font_manager import FontProperties
from scipy import stats

plot_markers = ('D', 'o', '^', 's', 'p', 'd', 'h', '8', r'$\lambda$', '_', '*', 'x',
                'H', 'v', '<', '>', '1', '2', '3', '4', '.', '|', '+', ',',)

plot_linestyles = ('-', '--', '-.', ':')

# These are the "Tableau 20" colors as RGB.
# http://tableaufriction.blogspot.ro/2012/11/finally-you-can-use-tableau-data-colors.html
# http://www.randalolson.com/2014/06/28/how-to-make-beautiful-data-visualizations-in-python-with-matplotlib/
tableau20 = [(31, 119, 180), (174, 199, 232), (255, 127, 14), (148, 103, 189),
             (44, 160, 44), (227, 119, 194), (214, 39, 40), (255, 152, 150),
             (255, 187, 120), (197, 176, 213), (152, 223, 138), (196, 156, 148),
             (140, 86, 75), (247, 182, 210), (127, 127, 127), (199, 199, 199),
             (188, 189, 34), (219, 219, 141), (23, 190, 207), (158, 218, 229)]

# Scale the RGB values to the [0, 1] range, which is the format matplotlib accepts.
for i in range(len(tableau20)):
    r, g, b = tableau20[i]
    tableau20[i] = (r / 255., g / 255., b / 255.)


def cnv_ns_to_ms(ns):
    return ns / 1000000.0


class Results:
    ALGORITHM = 0
    DOMAIN = 1

    def __init__(self, parsedJson):
        self.configuration = (parsedJson['experimentConfiguration']['algorithmName'],
                              parsedJson['experimentConfiguration']['domainName'])
        self.generatedNodes = parsedJson['generatedNodes']
        self.expandedNodes = parsedJson['expandedNodes']
        self.actions = parsedJson['actions']
        self.time = cnv_ns_to_ms(parsedJson['goalAchievementTime'])


def translate_algorithm_name(alg_name):
    # Handle hat (^) names
    if "HAT" in alg_name:
        alg_name = re.sub(r"(.*)_(.*)_(HAT)(.*)", r"\1", alg_name) \
                   + re.sub(r"(.*)_(.*)_(HAT)(.*)", r"_$\\hat{\2}$", alg_name).lower() \
                   + re.sub(r"(.*)_(.*)_(HAT)(.*)", r"\4", alg_name)

    # Specific word formatting
    alg_name = alg_name.replace('DYNAMIC_MULTIPLE', 'DM')
    alg_name = alg_name.replace('STATIC_MULTIPLE', 'SM')
    alg_name = alg_name.replace('STATIC_SINGLE', 'SS')
    alg_name = alg_name.replace('DYNAMIC', 'Dynamic')
    alg_name = alg_name.replace('WEIGHTED', 'Weighted')
    alg_name = alg_name.replace('STATIC', 'Static')
    alg_name = alg_name.replace('MULTIPLE', 'Mutliple')
    alg_name = alg_name.replace('SINGLE', 'Single')
    alg_name = alg_name.replace('LSS_', 'LSS-')
    # Handle star (*) names
    alg_name = alg_name.replace('_STAR', '*')
    # Replace rest of underscores
    alg_name = alg_name.replace('_', ' ')
    return alg_name


def translate_domain_name(domain_name):
    if domain_name is "POINT_ROBOT_WITH_INERTIA":
        return "Double Integrator"
    # Replace underscores
    domain_name = domain_name.replace('_', ' ')
    # Convert case
    domain_name = domain_name.title()
    return domain_name


def translate_instance_name(instance_name, domain):
    if domain == "ACROBOT":
        return instance_name
    else:
        # Strip path and extension
        name = os.path.splitext(os.path.basename(instance_name))[0]

        # Strip ending numbers
        if re.match(r'(.*)_[0-9]*', name):
            name = name.split('_')[0]

        # Convert casing and return
        final = re.sub("([a-z])([A-Z])", "\g<1> \g<2>", name).title()

        # Special replacements
        if final == 'H':
            final = 'HBox'
        elif final == 'Barto-Big':
            final = 'Barto Large'

        return final


def translate_domain_instance_name(domain_name, instance_name):
    return translate_domain_name(domain_name) + " - " + translate_instance_name(instance_name, domain_name)


def median_confidence_intervals(data):
    if not data:  # empty
        return [0], [0], [0]
    bxpstats = cbook.boxplot_stats(data)
    confidence_intervals = [[], []]
    medians = []
    for stat in bxpstats:
        confidence_intervals[0].append(stat['cilo'])
        confidence_intervals[1].append(stat['cihi'])
        medians.append(stat['med'])
    confidence_intervals[0] = np.array(confidence_intervals[0])
    confidence_intervals[1] = np.array(confidence_intervals[1])
    return medians, medians - confidence_intervals[0], confidence_intervals[1] - medians


def mean_confidence_intervals(data):
    if not data:  # empty
        return [0], [0], [0]

    means = [np.mean(x) for x in data]
    safe_means = np.nan_to_num(means)

    std = np.nan_to_num([stats.sem(x) if len(x) > 1 else 0.0 for x in data])
    confidence_intervals = stats.t.interval(0.95, len(data) - 1, loc=safe_means, scale=std)
    confidence_intervals_low = np.array(
        [mean if math.isnan(ci) else ci for mean, ci in zip(means, confidence_intervals[0])])
    confidence_intervals_high = np.array(
        [mean if math.isnan(ci) else ci for mean, ci in zip(means, confidence_intervals[1])])
    return means, means - confidence_intervals_low, confidence_intervals_high - means


def save_plot(plot, filename, lgd=None):
    basename, ext = os.path.splitext(filename)
    if ext is '.pdf':
        pp = PdfPages(filename)
        if lgd:
            plot.savefig(pp, format='pdf', bbox_inches='tight', bbox_extra_artists=(lgd,))
        else:
            plot.savefig(pp, format='pdf', bbox_inches='tight')
        pp.close()
    else:
        # Try and save it
        if lgd:
            plot.savefig(filename, bbox_inches='tight', bbox_extra_artists=(lgd,))
        else:
            plot.savefig(filename, bbox_inches='tight')


def plot_gat_stacked_bars(data: dict, labels: list, title="", stats_type="median", log10=True):
    if stats_type is not "median" and stats_type is not "mean":
        print("invalid type passed to plotutils.plot_gat_stacked_bars (must be median or mean)")
        return None

    gat_name = "goalAchievementTime"
    idle_name = "idlePlanningTime"

    if gat_name not in data or idle_name not in data:
        print("Missing data for plotutils.plot_gat_stacked_bars")
        return None

    y_gat = data[gat_name]
    y_idle = data[idle_name]

    if not y_gat:  # empty
        print("No data provided to plotutils.plot_gat_stacked_bars")
        return
    if len(y_gat) != len(y_idle):
        print("WARNING: Uneven data passed to plotutils.plot_gat_stacked_bars")
    x = np.arange(1, len(y_gat) + 1) + 0.1

    if stats_type is "median":
        gat_stats, gat_stats_confidence_interval_low, gat_stats_confidence_interval_high = \
            median_confidence_intervals(y_gat)
        idle_stats, idle_stats_confidence_interval_low, idle_stats_confidence_interval_high = \
            median_confidence_intervals(y_idle)
    else:  # assured to be mean
        gat_stats, gat_stats_confidence_interval_low, gat_stats_confidence_interval_high = \
            mean_confidence_intervals(y_gat)
        idle_stats, idle_stats_confidence_interval_low, idle_stats_confidence_interval_high = \
            mean_confidence_intervals(y_idle)

    width = 0.35
    fig, ax = plt.subplots()

    gat_bars = ax.bar(x, gat_stats, width, color=tableau20[0], ecolor=tableau20[5], capsize=2,
                      label="Goal Achievement Time", edgecolor='none',
                      yerr=(gat_stats_confidence_interval_low, gat_stats_confidence_interval_high))
    init_bars = ax.bar(x, idle_stats, width, color=tableau20[1], ecolor=tableau20[3], capsize=2,
                       label="Idle Planning Time", edgecolor='none',
                       yerr=(idle_stats_confidence_interval_low, idle_stats_confidence_interval_high))

    # Set labels
    plt.title(title, fontsize=16)
    if log10:
        ax.set_yscale('symlog', basey=10)
        # ax.set_yscale('log', nonposy='clip')
        plt.ylabel("Time log10", fontsize=16)
    else:
        plt.ylabel("Time (ms)", fontsize=16)
    plt.xlabel("Algorithms", fontsize=16)
    ax.set_xticks(x + width / 2.)
    ax.set_xticklabels(labels, fontsize=14)
    ax.autoscale(tight=True)
    fig.autofmt_xdate()
    lgd = ax.legend(loc='best', frameon=False)

    plt.gcf().tight_layout()

    return lgd


def plot_node_count_bars(data, labels, title="", stats_type="median", log10=True):
    if stats_type is not "median" and stats_type is not "mean":
        print("invalid type passed to plotutils.plot_node_count_bars (must be median or mean)")
        return None

    generated_name = "generatedNodes"
    expanded_name = "expandedNodes"

    if generated_name not in data or expanded_name not in data:
        print("Missing data for plotutils.plot_node_count_bars")
        return None

    y_generated = data[generated_name]
    y_expanded = data[expanded_name]

    if not y_generated:  # empty
        print("No data provided to plotutils.plot_node_count_bars")
        return
    if len(y_generated) != len(y_expanded):
        print("WARNING: Uneven data passed to plotutils.plot_node_count_bars")

    x = np.arange(1, len(y_generated) + 1)

    if stats_type is "median":
        generated_stats, generated_confidence_interval_low, generated_confidence_interval_high = \
            median_confidence_intervals(y_generated)
        expanded_stats, expanded_confidence_interval_low, expanded_confidence_interval_high = \
            median_confidence_intervals(y_expanded)
    else:  # assured to be mean
        generated_stats, generated_confidence_interval_low, generated_confidence_interval_high = \
            mean_confidence_intervals(y_generated)
        expanded_stats, expanded_confidence_interval_low, expanded_confidence_interval_high = \
            mean_confidence_intervals(y_expanded)

    width = 0.35

    fig, ax = plt.subplots()
    generated_count_bars = ax.bar(x, generated_stats, width, color=tableau20[4], ecolor=tableau20[2], capsize=2,
                                  edgecolor='none',
                                  yerr=(generated_confidence_interval_low, generated_confidence_interval_high))
    expanded_count_bars = ax.bar(x + width, expanded_stats, width, color=tableau20[10], ecolor=tableau20[6], capsize=2,
                                 edgecolor='none',
                                 yerr=(expanded_confidence_interval_low, expanded_confidence_interval_high))

    # Set labels
    plt.title(title, fontsize=16)
    plt.xlabel("Algorithms", fontsize=16)
    if log10:
        ax.set_yscale('symlog', basey=10)
        plt.ylabel("Node Count log10", fontsize=16)
    else:
        plt.ylabel("Node Count", fontsize=16)
    ax.set_xticks(x + width)
    ax.set_xticklabels(labels, fontsize=14)
    ax.autoscale(tight=True)
    fig.autofmt_xdate()
    lgd = ax.legend((generated_count_bars, expanded_count_bars), ('Expanded', 'Generated'), loc='best', frameon=False)

    # Add numbers to top of bars
    def autolabel(rects):
        for rect in rects:
            height = rect.get_height()
            ax.text(rect.get_x() + rect.get_width() / 2., 1.0 * height, '%d' % int(height), ha='center', va='bottom')

    # autolabel(med_bars)
    # autolabel(mean_bars)
    plt.gcf().tight_layout()

    return lgd


def plot_gat_bars(data, labels, title=""):
    y = data
    if not y:  # empty
        print("No data provided to plotutils.plot_gat_boxplots")
        return
    x = np.arange(1, len(y) + 1)
    med, med_confidence_interval_low, med_confidence_interval_high = median_confidence_intervals(y)
    mean, mean_confidence_interval_low, mean_confidence_interval_high = mean_confidence_intervals(y)
    width = 0.35

    fig, ax = plt.subplots()
    med_bars = ax.bar(x, med, width, color=tableau20[0], ecolor=tableau20[3], capsize=2,
                      yerr=(med_confidence_interval_low, med_confidence_interval_high))
    mean_bars = ax.bar(x + width, mean, width, color=tableau20[1], ecolor=tableau20[3], capsize=2,
                       yerr=(mean_confidence_interval_low, mean_confidence_interval_high))

    # Set labels
    plt.title(title)
    plt.ylabel("Goal Achievement Time (ms)")
    plt.xlabel("Algorithms")
    ax.set_xticks(x + width)
    ax.set_xticklabels(labels)
    fig.autofmt_xdate()
    lgd = ax.legend((med_bars, mean_bars), ('Median', 'Mean'), loc='best', frameon=False)

    ax.autoscale(tight=True)

    # Set ylims so we aren't at the top of the graph space for even data
    # low = min(min(y))
    # high = max(max(y))
    # plt.ylim([math.ceil(low - 0.5 * (high - low)), math.ceil(high + 0.5 * (high - low))])

    # Add numbers to top of bars
    def autolabel(rects):
        for rect in rects:
            height = rect.get_height()
            ax.text(rect.get_x() + rect.get_width() / 2., 1.0 * height, '%d' % int(height), ha='center', va='bottom')

    # autolabel(med_bars)
    # autolabel(mean_bars)
    plt.gcf().tight_layout()

    return lgd


def plot_gat_boxplots(data, labels, title="", showviolin=False):
    y = data
    if not y:  # empty
        print("No data provided to plotutils.plot_gat_boxplots")
        return
    med, confidence_interval_low, confidence_interval_high = median_confidence_intervals(y)

    fig, ax = plt.subplots()
    plt.boxplot(y, notch=False, labels=labels)
    # Plot separate error bars without line to show median confidence intervals
    x = np.arange(1, len(y) + 1)
    plt.errorbar(x, med, yerr=(confidence_interval_low, confidence_interval_high), fmt='none',
                 linewidth=3)

    if showviolin:
        mean, mean_confidence_interval_low, mean_confidence_interval_high = mean_confidence_intervals(y)
        plt.violinplot(y, showmeans=True, showmedians=True)
        plt.errorbar(x, mean, yerr=(mean_confidence_interval_low, mean_confidence_interval_high), fmt='none',
                     linewidth=3, color='g')

    plt.title(title)
    plt.ylabel("Goal Achievement Time (ms)")
    plt.xlabel("Algorithms")
    fig.autofmt_xdate()

    ax.autoscale(tight=True)
    # ymin, ymax = plt.ylim()
    # plt.ylim(ymin - 0.1, ymax + 0.1)
    plt.gcf().tight_layout()

    return None


def array_is_all_zero(arr):
    is_all_zero = True
    for val in arr:
        if val != 0:
            is_all_zero = False
            break
    return is_all_zero


# TODO add factor A*
def plot_gat_duration_error(data_dict, astar_data, action_durations, title="", log10=True):
    markers = itertools.cycle(plot_markers)
    linestyles = itertools.cycle(plot_linestyles)

    handles = []
    astar_gat_per_duration = astar_data
    if not astar_gat_per_duration:  # empty
        print("No data for A*")
    # astar_gat_per_duration_means, astar_confidence_interval_low, astar_confidence_interval_high = \
    #     mean_confidence_intervals(astar_gat_per_duration)
    x = np.arange(1, len(action_durations) + 1)

    fig = plt.figure()
    ax = plt.subplot()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.tick_params(direction='out')
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()

    # Plot for each provided algorithm
    color_index = 0
    for algorithm, algorithm_gat_per_duration in data_dict.items():
        if not algorithm_gat_per_duration:  # empty
            print("No data for " + algorithm)
            continue
        # print(algorithm_gat_per_duration)
        # if log10:
        #     algorithm_gat_per_duration = [np.log10(gat) for gat in algorithm_gat_per_duration]
        algorithm_gat_per_duration_mean, algorithm_confidence_interval_low, algorithm_confidence_interval_high = \
            mean_confidence_intervals(algorithm_gat_per_duration)
        data_mask = np.isfinite(algorithm_gat_per_duration_mean)

        masked_x = x[data_mask]
        masked_data = np.array(algorithm_gat_per_duration_mean)[data_mask]
        masked_confidence_low = algorithm_confidence_interval_low[data_mask]
        masked_confidence_high = algorithm_confidence_interval_high[data_mask]
        label = translate_algorithm_name(algorithm)

        handle = plt.errorbar(masked_x, masked_data, label=label, color=tableau20[color_index], markeredgecolor='none',
                              linestyle=next(linestyles), marker=next(markers), clip_on=False,
                              yerr=(masked_confidence_low, masked_confidence_high))
        handles.append((handle, label, masked_data.tolist()))
        color_index += 1

    # Set labels, sort legend by mean value
    handles = sorted(handles, key=lambda handle: handle[2], reverse=True)
    ordered_handles = [a[0] for a in handles]
    ordered_labels = [a[1] for a in handles]
    font_properties = FontProperties()
    font_properties.set_size('small')
    lgd = plt.legend(handles=ordered_handles, labels=ordered_labels, prop=font_properties, ncol=2, frameon=False,
                     # loc="upper center",bbox_to_anchor=(0.5, -0.1)
                     loc='best'
                     )
    plt.xticks(x, [duration if duration - int(duration) > 0 else int(duration) for duration in action_durations])
    plt.title(title)
    if log10:
        plt.yscale('symlog', basey=10)
        # plt.yscale('log', basey=10, nonposy='clip')
        plt.ylabel("Goal Achievement Time log10")
    else:
        plt.ylabel("Goal Achievement Time (ms)")
    plt.xlabel("Action Duration (ms)")
    ax.autoscale(tight=True)

    plt.gcf().tight_layout()

    return lgd
