# metronome

Real time search library implemented in C++

# Requirements

## Metronome C++

* C++ 17 (Tested with Clang and GCC)
* Cmake
* Boost
* Gephi with Graph Streaming (optional for visualizaion)

## Experiment execution and plotting

* Python 3.5 + NumPy
* PyMongo

# features

Search domains including VacuumWorld, Traffic, TestDomain

Algorithm set that makes use of real time search paradigms 

# How to run

1. Generate makefiles with Cmake and compile the project
2. Metronome requires the resource folder as the first argument and a configuration file. The latter can be provided
as the second argument or as standard input.

Example configuration:

"timeLimit" : NumberLong(150000000000),
"domainPath" : "input/vacuum/h_400.vw",
"domainInstanceName" : "input/vacuum/h_400.vw",
"actionDuration" : NumberLong(6000000),
"domainName" : "GRID_WORLD",
"terminationType" : "time",
"algorithmName" : "A_STAR"
