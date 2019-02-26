# Metronome

Real time search library implemented in C++

# Requirements

## Metronome C++

* C++ 17 (Tested with Clang and GCC)
* Cmake
* Boost
* Gephi with Graph Streaming (optional for visualizaion)

## Experiment execution and plotting

* Python 3.6 (packages: numpy, pandas, distlre, jupyterlab)

# features

Search domains including VacuumWorld, Traffic, TestDomain

Algorithm set that makes use of real time search paradigms 

# How to run

1. Generate makefiles with Cmake and compile the project
2. Metronome requires the resource folder as the first argument and a configuration file. The latter can be provided
as the second argument or as standard input.

Example configuration:
{
  "timeLimit": 300000000000,
  "domainInstanceName": "Manual test instance",
  "actionDuration": 100000000000,
  "domainName": "GRID_WORLD",
  "domainPath": "input/vacuum/uniform40/1k1k/uniform1000_1000-0",
  "terminationType": "TIME",
  "algorithmName": "TIME_BOUNDED_A_STAR",
  "lookaheadType": "DYNAMIC",
  "commitmentStrategy": "SINGLE",
  "weight": 1.0
} 
