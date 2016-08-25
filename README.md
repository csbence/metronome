# metronome

Real time search library implemented in C++

# Requirements

* Clang 3.8
* Cmake
* Python 3.5 + NumPy


# features

Search domains including VaccumWorld, Traffic, TestDomain

Algorithm set that makes use of real time search paradigms 

# todo

Finish VacuumWorld and Traffic representation

Implement real time A* and its extentions (LSS-LRTA*)


Input format:

"timeLimit" : NumberLong(150000000000),
"domainPath" : "input/vacuum/h_400.vw",
"domainInstanceName" : "input/vacuum/h_400.vw",
"actionDuration" : NumberLong(6000000),
"domainName" : "GRID_WORLD",
"terminationType" : "time",
"algorithmName" : "A_STAR"
