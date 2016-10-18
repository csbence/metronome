#!/usr/bin/python3

import copy
import getopt
import os
import sys
import warnings
from enum import Enum
from subprocess import call

import matplotlib.pyplot as plt
from pymongo import MongoClient

import plotutils

warnings.filterwarnings("ignore", message=".*Source ID.*", module="matplotlib")
warnings.filterwarnings("ignore", message="Attempting to set identical bottom==top results", module="matplotlib")
warnings.filterwarnings("ignore", message="Mean of empty slice", module="numpy")
warnings.filterwarnings("ignore", message="invalid value encountered in double_scalars", module="numpy")
warnings.filterwarnings("ignore", message="invalid value encountered in multiply", module="scipy")

file_format = "pdf"


class GraphType(Enum):
    all = 1
    gatPerDuration = 2
    gatBoxPlot = 3
    gatBars = 4
    gatViolin = 5
    gatStacked = 6


def get_configuration_plot_name(owner: str, configuration: dict):
    return owner + "_" + concatenate_configuration(configuration)


def concatenate_configuration(configuration: dict, separator: str = "_", include_names: bool = False):
    name = ""
    sorted_keys = sorted(configuration, reverse=True)
    for key in sorted_keys:
        value = configuration[key]
        to_concatenate = ""
        if include_names:
            to_concatenate += str(key) + ":" + str(value)
        else:
            to_concatenate += str(value)

        if name:
            name += separator + to_concatenate
        else:
            name += to_concatenate
    return name


def get_plot_algorithm_names():
    names = {}
    for algorithm, configurations in all_algorithms.items():
        if not configurations:
            names[algorithm] = algorithm
        else:
            for configuration in configurations:
                names[get_configuration_plot_name(algorithm, configuration)] = algorithm
    return names


def get_pri_configurations():
    # pri_action_fractions = [1.0, 2.0]
    # pri_num_actions = [3, 5, 7]
    # pri_state_fractions = [0.25, 0.5]
    pri_action_fractions = [1.0]
    pri_num_actions = [3]
    pri_state_fractions = [0.5]
    configurations = []
    for num_actions in pri_num_actions:
        for action_fraction in pri_action_fractions:
            for state_fraction in pri_state_fractions:
                configurations.append({"numActions": num_actions,
                                       "actionFraction": action_fraction,
                                       "stateFraction": state_fraction})
    return configurations


script = os.path.basename(sys.argv[0])
options = "hs:qa:d:i:t:c:"
default_graph_type = GraphType.gatPerDuration
all_action_durations = (
6000000, 10000000, 20000000, 40000000, 80000000, 160000000, 320000000, 640000000, 850000000, 960000000, 1070000000,
1280000000)
# all_action_durations = (850000000, 960000000, 1070000000, 1280000000)
all_action_durations_ms = [plotutils.cnv_ns_to_ms(duration) for duration in all_action_durations]
# all_algorithms = ["A_STAR", "ARA_STAR", "RTA_STAR", "LSS_LRTA_STAR", "DYNAMIC_F_HAT"]
ss_configuration = {"timeBoundType": "STATIC", "commitmentStrategy": "SINGLE"}
sm_configuration = {"timeBoundType": "STATIC", "commitmentStrategy": "MULTIPLE"}
dm_configuration = {"timeBoundType": "DYNAMIC", "commitmentStrategy": "MULTIPLE"}
all_algorithms = {"A_STAR": [], "ARA_STAR": [], "RTA_STAR": [],
                  "LSS_LRTA_STAR": [ss_configuration, sm_configuration, dm_configuration],
                  "DYNAMIC_F_HAT": [ss_configuration, sm_configuration, dm_configuration]}
plot_algorithm_names = get_plot_algorithm_names()
# all_domains = ["GRID_WORLD", "SLIDING_TILE_PUZZLE_4", "ACROBOT", "POINT_ROBOT", "POINT_ROBOT_WITH_INERTIA", "RACETRACK"]
all_domains = {"GRID_WORLD": [],
               "SLIDING_TILE_PUZZLE_4": [],
               "ACROBOT": [],
               "POINT_ROBOT": [],
               "POINT_ROBOT_WITH_INERTIA": get_pri_configurations(),
               "RACETRACK": []}
all_acrobot_instances = [
    "0.07-0.07",
    "0.08-0.08",
    "0.09-0.09",
    "0.1-0.1",
    "0.3-0.3"
]
all_dylan_instances = [
    "dylan/cups",
    "dylan/slalom",
    "dylan/uniform",
    "dylan/wall"
]
big_uniform_instances = [
    "random1k",
    "randomShapes1k",
    "randomNoisy1k"
]
special_grid_instances = [
    "openBox_800",
    "squiggle_800",
    "slalom_03",
    "slalom_04",
    "openBox_400",
    "openBox_25",
    "squiggle",
    "h_400",
    "hole_400"
]
all_racetrack_instances = [
    "input/racetrack/barto-big.track",
    "input/racetrack/barto-small.track",
    "input/racetrack/hansen-bigger.track",
    "input/racetrack/long.track"
]
sliding_tile_4_map_root = "input/tiles/korf/4/all"
all_sliding_tile_4_instances = [2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21, 23, 24, 25, 26, 27, 28,
                                29, 30, 31, 32, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 47, 48, 49, 50,
                                51, 52, 53, 54, 55, 56, 57, 58, 59, 62, 64, 65, 66, 67, 68, 69, 70, 71, 73, 75,
                                76, 77, 78, 79, 80, 81, 83, 84, 85, 86, 87, 89, 92, 93, 94, 95, 96, 97, 98, 99, 100]


def usage():
    print("usage:")
    print("{} [{}]".format(script, options))
    print("options:")
    print("  h          print this usage info")
    print("  a<alg>     specify an algorithm, one per switch")
    print("  d<domain>  specify domain")
    print("  i<name>    specify instance name")
    print("  c<nano>    specify action duration")
    print("  s<file>    save to file")
    print("  t<type>    specify type of plot; default {}".format(default_graph_type.name))
    print("  q          quiet mode; no logging or graph showing")
    print("valid graph types:")
    for graph_type in GraphType:
        print("  " + str(graph_type).replace("GraphType.", ""))
    print("valid action durations:")
    for action_duration in all_action_durations:
        print("  " + str(action_duration))


def open_connection():
    client = MongoClient('mongodb://aerials.cs.unh.edu:42830')
    client.rts.authenticate('rtsUser', 'VeLuvh4!', mechanism='SCRAM-SHA-1')
    return client.rts


def print_counts(db):
    configuration_status = db.command('collstats', 'experimentConfiguration')
    print('Configuration count: %d' % configuration_status['count'])
    task_status = db.command('collstats', 'experimentTask')
    print('Task count: %d' % task_status['count'])
    result_status = db.command('collstats', 'experimentResultV5_inertia')
    print('Result count: %d' % result_status['count'])
    # pprint.pprint(configuration_status, width=1)


def get_configuration_query(configuration):
    return "result.experimentConfiguration.{}".format(configuration)


def get_gat_per_duration_data(db, algorithm, domain, instance, planner_configuration: dict = None,
                              domain_configuration: dict = None):
    data_action_durations = []

    query = {
        "experimentConfiguration.domainName": domain,
        "experimentConfiguration.algorithmName": algorithm,
        "experimentConfiguration.domainPath": instance,

        "success": True
    }

    if planner_configuration:
        for name, value in planner_configuration.items():
            query[get_configuration_query(name)] = value

    if domain_configuration:
        for name, value in domain_configuration.items():
            query[get_configuration_query(name)] = value

    for action_duration in all_action_durations:
        query[get_configuration_query("actionDuration")] = action_duration
        data_tiles = db.experimentResultV5_inertia.find(query)

        times_for_durations = []
        for result in data_tiles:
            times_for_durations.append(plotutils.cnv_ns_to_ms(result['result']['goalAchievementTime']))

        # if times_for_durations:  # not empty
        data_action_durations.append(times_for_durations)

    return data_action_durations


def gather_gat_data(db, query, old_idle_time: bool = False):
    data_tiles = db.experimentResultV5_inertia.find(query)

    data = []
    idle_data = []
    for result in data_tiles:
        data.append(plotutils.cnv_ns_to_ms(result['result']['goalAchievementTime']))
        if old_idle_time:
            algorithm = query["result.experimentConfiguration.algorithmName"]
            if algorithm is "A_STAR" or algorithm is "ARA_STAR" or algorithm is "WEIGHTED_A_STAR":
                idle_data.append(plotutils.cnv_ns_to_ms(result['result']['planningTime']))
            elif algorithm is "LSS_LRTA_STAR" or algorithm is "DYNAMIC_F_HAT" or algorithm is "RTA_STAR":
                idle_data.append(plotutils.cnv_ns_to_ms(result['result']['experimentConfiguration']['actionDuration']))
            else:
                print("Don't know how to get idle planning time for {}!!!!".format(algorithm))
        else:
            idle_data.append(plotutils.cnv_ns_to_ms(result['result']['idlePlanningTime']))
    return data, idle_data


def gather_node_data(db, query):
    data_tiles = db.experimentResultV5_inertia.find(query)

    generated_data = []
    expanded_data = []
    for result in data_tiles:
        generated_data.append(result['result']['expandedNodes'])
        expanded_data.append(result['result']['generatedNodes'])
    return generated_data, expanded_data


# TODO get rid of code duplication
def get_node_data(db, algorithms, domain, instance, action_duration, domain_configuration: dict = None):
    generated_data = []
    expanded_data = []
    labels = []
    indices = {}

    def add_data(generated_subdata, expanded_subdata, algorithm, configuration=None):
        labelsUpdated = False
        if generated_subdata:  # not empty
            generated_data.append(generated_subdata)
            if configuration is not None:
                labels.append(
                    plotutils.translate_algorithm_name(get_configuration_plot_name(algorithm, configuration)))
            else:
                labels.append(plotutils.translate_algorithm_name(algorithm))
            labelsUpdated = True
        if expanded_subdata:
            expanded_data.append(expanded_subdata)
        return labelsUpdated

    query = {
        "result.experimentConfiguration.domainName": domain,
        "result.experimentConfiguration.domainInstanceName": instance,
        "result.experimentConfiguration.actionDuration": int(action_duration),
        "result.success": True
    }

    def gather_data_per_algorithm():
        for algorithm in algorithms:
            planner_configurations = all_algorithms[algorithm]
            query[get_configuration_query("algorithmName")] = algorithm
            if planner_configurations:
                for planner_configuration in planner_configurations:
                    for name, value in planner_configuration.items():
                        query[get_configuration_query(name)] = value

                    generated_subdata, expanded_subdata = gather_node_data(db, query)
                    labelsUpdated = add_data(generated_subdata, expanded_subdata, algorithm, planner_configuration)
                    if labelsUpdated:
                        indices[get_configuration_plot_name(algorithm, planner_configuration)] = len(labels) - 1

                    # Make sure the previous configuration is not used in the next query
                    for name, value in planner_configuration.items():
                        del query[get_configuration_query(name)]
            else:
                generated_subdata, expanded_subdata = gather_node_data(db, query)
                labelsUpdated = add_data(generated_subdata, expanded_subdata, algorithm)
                if labelsUpdated:
                    indices[algorithm] = len(labels) - 1

    if domain_configuration:
        for name, value in domain_configuration.items():
            query[get_configuration_query(name)] = value

    gather_data_per_algorithm()

    # Make sure the previous configuration is not used in the next query
    if domain_configuration:
        for name, value in domain_configuration.items():
            del query[get_configuration_query(name)]

    return {"generatedNodes": generated_data, "expandedNodes": expanded_data}, labels, indices


def get_gat_data(db, algorithms, domain, instance, action_duration, domain_configuration: dict = None,
                 old_idle_time: bool = False):
    gat_data = []
    idle_planning_data = []
    labels = []
    indices = {}

    def add_data(data, idle_data, algorithm, configuration=None):
        labelsUpdated = False
        if data:  # not empty
            gat_data.append(data)
            if configuration is not None:
                labels.append(
                    plotutils.translate_algorithm_name(get_configuration_plot_name(algorithm, configuration)))
            else:
                labels.append(plotutils.translate_algorithm_name(algorithm))
            labelsUpdated = True
        if idle_data:
            idle_planning_data.append(idle_data)
        return labelsUpdated

    query = {
        "result.experimentConfiguration.domainName": domain,
        "result.experimentConfiguration.domainInstanceName": instance,
        "result.experimentConfiguration.actionDuration": int(action_duration),
        "result.success": True
    }

    def gather_data_per_algorithm():
        for algorithm in algorithms:
            planner_configurations = all_algorithms[algorithm]
            query[get_configuration_query("algorithmName")] = algorithm
            if planner_configurations:
                for planner_configuration in planner_configurations:
                    for name, value in planner_configuration.items():
                        query[get_configuration_query(name)] = value

                    data, idle_data = gather_gat_data(db, query, old_idle_time)
                    labelsUpdated = add_data(data, idle_data, algorithm, planner_configuration)
                    if labelsUpdated:
                        indices[get_configuration_plot_name(algorithm, planner_configuration)] = len(labels) - 1

                    # Make sure the previous configuration is not used in the next query
                    for name, value in planner_configuration.items():
                        del query[get_configuration_query(name)]
            else:
                data, idle_data = gather_gat_data(db, query, old_idle_time)
                labelsUpdated = add_data(data, idle_data, algorithm)
                if labelsUpdated:
                    indices[algorithm] = len(labels) - 1

    if domain_configuration:
        for name, value in domain_configuration.items():
            query[get_configuration_query(name)] = value

    gather_data_per_algorithm()

    # Make sure the previous configuration is not used in the next query
    if domain_configuration:
        for name, value in domain_configuration.items():
            del query[get_configuration_query(name)]

    return {"goalAchievementTime": gat_data, "idlePlanningTime": idle_planning_data}, labels, indices


# TODO add factor A*
def plot_gat_duration_error(db, algorithms, domain, instance):
    # Gather required A* data
    astar_gat_per_duration = get_gat_per_duration_data(db, "A_STAR", domain, instance)
    algorithm_data = {}
    # Plot for each provided algorithm
    for algorithm in algorithms:
        algorithm_data[algorithm] = get_gat_per_duration_data(db, algorithm, domain, instance)
    plotutils.plot_gat_duration_error(algorithm_data, astar_gat_per_duration, all_action_durations_ms,
                                      title=plotutils.translate_domain_name(domain) + " - " + instance)


def plot_gat_boxplots(db, algorithms, domain, instance, action_duration, showviolin=False):
    data, labels, _ = get_gat_data(db, algorithms, domain, instance, action_duration)
    y = data["goalAchievementTime"]
    plotutils.plot_gat_boxplots(y, labels, showviolin=showviolin,
                                title=plotutils.translate_domain_name(domain) + "-" + instance)


def plot_gat_bars(db, algorithms, domain, instance, action_duration):
    data, labels, _ = get_gat_data(db, algorithms, domain, instance, action_duration)
    y = data["goalAchievementTime"]
    plotutils.plot_gat_bars(y, labels, title=plotutils.translate_domain_name(domain) + "-" + instance)


def plot_gat_stacked(db, algorithms, domain, instance, action_duration, old_idle_time: bool = False):
    data, labels, _ = get_gat_data(db, algorithms, domain, instance, action_duration, old_idle_time=old_idle_time)
    plotutils.plot_gat_stacked_bars(data, labels, title=plotutils.translate_domain_name(domain) + "-" + instance)


def do_plot(file_header, file_suffix, plot):
    # file_header = "{}_{}".format(domain, instance_file_name)
    filename = "plots/{}_{}.{}".format(file_header, file_suffix, file_format)
    lgd = plot()
    plotutils.save_plot(plt, filename, lgd)
    plt.close('all')
    return "![{}]({})\n\n\\clearpage\n\n".format(file_header, filename)


def save_to_file(filename, text):
    text_file = open(filename, "w")
    text_file.write(text)
    text_file.close()


# http://stackoverflow.com/questions/377017/test-if-executable-exists-in-python#answer-377028
def which(program):
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

    return None


def remove_empty_data(dict: dict):
    for key, data in list(dict.items()):
        has_data = False
        for subdata in data:
            if subdata:
                has_data = True
                break
        if not has_data:
            del dict[key]


def remove_algorithms(data: dict, labels: list, indices: dict, plot_without: tuple):
    # Plot without exceptions
    removed = ""
    # print("----------------------------------")
    # print(data)
    # print(labels)
    # print(indices)
    # print("========")
    local_indices = copy.deepcopy(indices)
    for algorithm in plot_without:
        # print(algorithm)
        should_remove = True
        if algorithm in local_indices:
            for value in data.values():
                if local_indices[algorithm] >= len(value):
                    should_remove = False
                    break
        else:
            should_remove = False
        # print(should_remove)
        if should_remove:
            for value in data.values():
                value.pop(local_indices[algorithm])
            labels.pop(local_indices[algorithm])
            if not removed:  # empty
                removed += algorithm
            else:
                removed += "_" + algorithm
            # Update indices
            for alg, index in local_indices.items():
                if index > local_indices[algorithm]:
                    local_indices[alg] -= 1
            del local_indices[algorithm]
    # print(labels)
    #         print(local_indices)
    # print("----------------------------------")
    return removed


def plot_all_for_domain(db, domain: str, instances: list, plot_average: bool = False, average_only: bool = False,
                        markdown_summary: bool = True, error_only: bool = False, plot_without: tuple =
                        ("RTA_STAR", "LSS_LRTA_STAR_STATIC_MULTIPLE", "DYNAMIC_F_HAT_STATIC_MULTIPLE"),
                        old_idle_time: bool = False, log10: bool = True):
    if average_only:
        markdown_summary = False
        plot_average = True

    # translated_domain_name = plotutils.translate_domain_name(domain)
    domain_configurations = all_domains[domain]

    all_error_data = {}
    all_astar_error_data = []
    markdown_document = "# Domain: {}\n\n".format(domain)

    for index, plot_algorithm_name in enumerate(plot_algorithm_names.keys()):
        all_error_data[plot_algorithm_name] = []
        for _ in all_action_durations:
            all_error_data[plot_algorithm_name].append([])
    for _ in all_action_durations:
        all_astar_error_data.append([])

    def plot_domain_instance(instance, domain_configuration):
        instance_file_name = instance.replace("/", "_")
        plot_title = plotutils.translate_domain_instance_name(domain, instance)  # translated_domain_name + " - " + instance
        plots_markdown = ""

        instance_level = "#"
        if domain_configuration:
            instance_level += "#"
            instance_file_name += "_" + concatenate_configuration(domain_configuration)

        if not quiet:
            print("Processing {} - {}".format(domain, instance))

        # Gather error data for each algorithm
        astar_gat_per_duration = get_gat_per_duration_data(db, "A_STAR", domain, instance, domain_configuration)
        algorithm_gat_per_duration = {}
        for algorithm, configurations in all_algorithms.items():
            if not configurations:
                algorithm_gat_per_duration[algorithm] = \
                    get_gat_per_duration_data(db, algorithm, domain, instance, domain_configuration)
            else:
                for configuration in configurations:
                    algorithm_gat_per_duration[get_configuration_plot_name(algorithm, configuration)] = \
                        get_gat_per_duration_data(db, algorithm, domain, instance, configuration, domain_configuration)
        remove_empty_data(algorithm_gat_per_duration)

        # Store data to compute average later
        for algorithm, values in algorithm_gat_per_duration.items():
            count = 0
            for val in values:
                if val:
                    all_error_data[algorithm][count].append(val[0])
                count += 1

        count = 0
        for val in astar_gat_per_duration:
            if val:
                all_astar_error_data[count].append(val[0])
            else:
                all_astar_error_data[count].append([])
            count += 1

        # Do individual plots if average_only not specified
        if not average_only:
            # Errorbar plot
            if algorithm_gat_per_duration:
                if not quiet:
                    print("Plotting error plot: {} - {} - {}".format(domain, instance, domain_configuration))
                file_header = "{}_{}".format(domain, instance_file_name)
                plots_markdown += do_plot(file_header, "error", lambda:
                plotutils.plot_gat_duration_error(algorithm_gat_per_duration, astar_gat_per_duration,
                                                  all_action_durations_ms, title=plot_title, log10=log10))

                # Also plot errorbar without exception algorithms
                removed = ""
                for algorithm in plot_without:
                    algorithm_data = algorithm_gat_per_duration.pop(algorithm, None)
                    if algorithm_data is not None:
                        if not removed:  # empty
                            removed += algorithm
                        else:
                            removed += "_" + algorithm

                # Plot it if there was anything to remove and there is still data left over
                if removed and algorithm_gat_per_duration:
                    if not quiet:
                        print("Plotting error plot: {} - {} - {} without {}".format(domain, instance,
                                                                                    domain_configuration, removed))
                    file_header = "{}_{}_NO_{}".format(domain, instance_file_name, removed)
                    plots_markdown += do_plot(file_header, "error", lambda:
                    plotutils.plot_gat_duration_error(algorithm_gat_per_duration, astar_gat_per_duration,
                                                      all_action_durations_ms, title=plot_title))

            if not error_only:
                for action_duration in all_action_durations:
                    action_plot_title = plot_title + " - " + str(int(plotutils.cnv_ns_to_ms(action_duration))) + " ms"
                    file_header = "{}_{}_{}".format(domain, instance_file_name, action_duration)
                    # Gather GAT data
                    gat_data, gat_labels, gat_indices = get_gat_data(db, all_algorithms.keys(), domain, instance,
                                                                     action_duration, domain_configuration,
                                                                     old_idle_time)
                    y_gat = gat_data['goalAchievementTime']
                    y_idle = gat_data['idlePlanningTime']

                    # Gather node data
                    node_data, node_labels, node_indices = get_node_data(db, all_algorithms.keys(), domain, instance,
                                                                         action_duration, domain_configuration)
                    y_generated = node_data['generatedNodes']
                    y_expanded = node_data['expandedNodes']

                    if y_gat or y_generated:
                        action_duration_level = instance_level + "#"
                        plots_markdown += "{} Action Duration: {} ns\n\n".format(action_duration_level, action_duration)

                    def plot_stacked(data, labels, file_header=file_header, print_suffix=""):
                        stacked_markdown = ""
                        if not quiet:
                            msg = "Plotting stacked bars: {} - {} - {} - {}".format(domain, instance, action_duration,
                                                                                    domain_configuration)
                            if print_suffix:
                                msg += " - {}".format(print_suffix)
                            print(msg)
                        stacked_markdown += do_plot(file_header, "stacked", lambda:
                        plotutils.plot_gat_stacked_bars(data, labels, title=action_plot_title, log10=log10))
                        return stacked_markdown

                    def plot_node_bars(data, labels, file_header=file_header, print_suffix=""):
                        nodes_markdown = ""
                        if not quiet:
                            msg = "Plotting node bars: {} - {} - {} - {}".format(domain, instance, action_duration,
                                                                                 domain_configuration)
                            if print_suffix:
                                msg += " - {}".format(print_suffix)
                            print(msg)
                        nodes_markdown += do_plot(file_header, "nodes", lambda:
                        plotutils.plot_node_count_bars(data, labels, title=action_plot_title, log10=log10))
                        return nodes_markdown

                    if y_gat and y_idle:
                        plots_markdown += plot_stacked(gat_data, gat_labels)

                        removed = remove_algorithms(gat_data, gat_labels, gat_indices, plot_without)
                        # Plot it if there was anything to remove and there is still data left over
                        if removed and gat_data['goalAchievementTime'] and gat_data["idlePlanningTime"]:
                            new_file_header = "{}_{}_{}_NO_{}".format(domain, instance_file_name, action_duration,
                                                                      removed)
                            suffix = "without {}".format(removed)
                            plots_markdown += plot_stacked(gat_data, gat_labels, file_header=new_file_header,
                                                           print_suffix=suffix)

                    if y_generated and y_expanded:
                        plots_markdown += plot_node_bars(node_data, node_labels)

                        removed = remove_algorithms(node_data, node_labels, node_indices, plot_without)
                        # Plot it if there was anything to remove and there is still data left over
                        if removed and node_data['generatedNodes'] and node_data["expandedNodes"]:
                            new_file_header = "{}_{}_{}_NO_{}".format(domain, instance_file_name, action_duration,
                                                                      removed)
                            suffix = "without {}".format(removed)
                            plots_markdown += plot_node_bars(node_data, node_labels, file_header=new_file_header,
                                                             print_suffix=suffix)

        if not plots_markdown:
            return ""
        else:
            instance_markdown_document = "{} Instance: {}\n\n".format(instance_level, instance)
            return instance_markdown_document + plots_markdown

    if domain_configurations:
        for domain_configuration in domain_configurations:
            instance_markdown = ""
            for instance in instances:
                instance_markdown += plot_domain_instance(instance, domain_configuration)
            if instance_markdown:
                markdown_document += "# Configuration {}\n\n".format(
                    concatenate_configuration(domain_configuration, separator=" ", include_names=True))
                markdown_document += instance_markdown
    else:
        for instance in instances:
            markdown_document += plot_domain_instance(instance, None)

    # Produce markdown file and convert to pdf if pandoc present
    if markdown_summary:
        if not quiet:
            print("Saving markdown file")
        file_header = "plots/{}_plots".format(domain)
        markdown_file = "{}.md".format(file_header)
        pdf_file = "{}.pdf".format(file_header)
        save_to_file(markdown_file, markdown_document)
        pandoc = which("pandoc")
        if pandoc:
            call([pandoc, "-o", pdf_file, markdown_file])

    # Plot average data
    remove_empty_data(all_error_data)
    if plot_average:
        if not quiet:
            print("Plotting {} averages".format(domain))
        lgd = plotutils.plot_gat_duration_error(all_error_data, all_astar_error_data, all_action_durations_ms,
                                                title="{} data over all instances".format(
                                                    plotutils.translate_domain_name(domain)))
        plotutils.save_plot(plt, "plots/{}_average.pdf".format(domain), lgd)
        plt.close('all')

        # Plot averages without exception algorithms
        removed = ""
        for algorithm in plot_without:
            algorithm_data = all_error_data.pop(algorithm, None)
            if algorithm_data is not None:
                if not removed:  # empty
                    removed += algorithm
                else:
                    removed += "_" + algorithm
        if removed:
            lgd = plotutils.plot_gat_duration_error(all_error_data, all_astar_error_data, all_action_durations_ms,
                                                    title="{} data over all instances no {}".format(
                                                        plotutils.translate_domain_name(domain), removed))
            plotutils.save_plot(plt, "plots/{}_NO_{}_average.pdf".format(domain, removed), lgd)
            plt.close('all')


def get_all_grid_world_instances():
    instances = []
    for instance in all_dylan_instances:
        instances.append("input/vacuum/{}.vw".format(instance))
    # for instance in big_uniform_instances:
    #     instances.append("input/vacuum/{}.vw".format(instance))
    for instance in special_grid_instances:
        instances.append("input/vacuum/{}.vw".format(instance))
    return instances


def get_all_point_robot_instances():
    instances = []
    for instance in all_dylan_instances:
        instances.append("input/pointrobot/{}.pr".format(instance))
    for instance in special_grid_instances:
        instances.append("input/pointrobot/{}.pr".format(instance))
    return instances


def get_all_sliding_tile_instances():
    instances = []
    for instance in all_sliding_tile_4_instances:
        instances.append("{}/{}".format(sliding_tile_4_map_root, instance))
    return instances


def plot_all(db):
    if not os.path.exists("plots"):
        os.makedirs("plots")

    plot_all_for_domain(db, "GRID_WORLD", get_all_grid_world_instances())
    plot_all_for_domain(db, "POINT_ROBOT", get_all_point_robot_instances())
    plot_all_for_domain(db, "POINT_ROBOT_WITH_INERTIA", get_all_point_robot_instances())
    plot_all_for_domain(db, "RACETRACK", all_racetrack_instances)
    plot_all_for_domain(db, "ACROBOT", all_acrobot_instances, plot_average=True)
    plot_all_for_domain(db, "SLIDING_TILE_PUZZLE_4", get_all_sliding_tile_instances(), plot_average=True)


if __name__ == '__main__':
    save_file = None
    quiet = False
    algorithms = []
    domain = None
    instance = None
    action_duration = all_action_durations[0]
    graph_type = default_graph_type

    try:
        opts, args = getopt.gnu_getopt(sys.argv[1:], options)
    except getopt.GetoptError as e:
        print("Getopt error: {0}".format(e.strerror))
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage()
            sys.exit(0)
        elif opt in ('-a', '--algorithm'):
            algorithms.append(arg)
        elif opt in ('-d', '--domain'):
            domain = arg
        elif opt in ('-i', '--instance'):
            instance = arg
        elif opt in ('-c', '--action'):
            action_duration = arg
        elif opt in ('-t', '--type'):
            graph_type = getattr(GraphType, arg)
        elif opt in ('-s', '--save'):
            save_file = arg
        elif opt in ('-q', '--quiet'):
            quiet = True
        else:
            print("invalid switch '%s'" % opt)
            usage()
            sys.exit(2)

    if graph_type is not GraphType.all:
        if not algorithms:
            print("Must provide at least 1 algorithm")
            usage()
            sys.exit(2)
        elif domain is None or instance is None:
            print("Must provide domain and instance")
            usage()
            sys.exit(2)

    db = open_connection()
    if not quiet:
        print_counts(db)

    plotter = {
        GraphType.all: lambda: plot_all(db),
        GraphType.gatPerDuration: lambda: plot_gat_duration_error(db, algorithms, domain, instance),
        GraphType.gatBoxPlot: lambda: plot_gat_boxplots(db, algorithms, domain, instance, action_duration),
        GraphType.gatBars: lambda: plot_gat_bars(db, algorithms, domain, instance, action_duration),
        GraphType.gatViolin: lambda: plot_gat_boxplots(db, algorithms, domain, instance, action_duration,
                                                       showviolin=True),
        GraphType.gatStacked: lambda: plot_gat_stacked(db, algorithms, domain, instance, action_duration)
    }[graph_type]

    plotter()

    # Save before showing since show resets the figures
    if save_file is not None:
        plotutils.save_plot(plt, save_file)

    if not quiet and graph_type is not GraphType.all:
        plt.show()
