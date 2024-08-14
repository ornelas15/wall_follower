# Wall Follower agent
Develop of 2 agents to command a simulated mobile robot. The objective of this challenge is to control the movement of a robot through an unknown closed path defined by a wall. The robot should follow the wall, keeping the wall on the right of the robot and not deviating too much from the wall. 

## Install

The source code was compiled with gcc/g++ - Gnu Project C/C++ Compiler
(gcc version  9.3.0) using the Qt libraries (release 5.12.8) on Ubuntu 20.04.

It is required to have the development version of gcc/g++, cmake, Qt libraries
release 5.x installed in the system prior to compilation.
On Ubuntu 20.04 run the following:
```bash
sudo apt-get install build-essential cmake qtmultimedia5-dev
```

Then in the repository base dir, execute:
```bash
cd ciberRatoTools
mkdir build
cd build
cmake ..
make
cd ..
```

For the Machine Learning agent install:
```bash
pip install stable-baselines3
```

## Run agent 1

To run the simulator, the agent and Viewer, execute (at the repository ciberRatoTools):
```bash
./start
```

## Run agent 2

To run the simulator, the agent and Viewer, execute (at the repository ciberRatoTools):
```bash
./start2
```

Press start on the Simulator after connecting to the viewer


# Author

* David Ornelas
  University of Aveiro,
  david.ornelas@ua.pt  
  

# CiberRato Robot Simulation Environment <br/> Universidade de Aveiro / IEETA 

## Information

CiberRato Robot Simulation Environment simulates the movement
of robots inside a labyrinth.  Robots objective is to go from their
starting position to beacon area and then return to their start position.

The MicroRato competition
[http://microrato.ua.pt/], held annually at Aveiro University, 
uses these these tools for its Explorer league.

## Contents

* simulator -           The simulator source code
* Viewer -              The Visualizer source code
* logplayer -           The logplayer source code
* GUISample -           Graphical robot agent (C++) source code
* robsample -           robot agent (C) source code
* jClient -             robot agent (Java) source code
* pClient -             robot agent (Python) source code
* Labs -                examples of labyrinths used in previous competitions
* startAll -            script that runs the simulator, the visualizer and 5 GUISamples
* startSimViewer -      script that runs the simulator and the Viewer


## Authors

* Nuno Lau,
  University of Aveiro,
  nunolau@ua.pt

* Artur C. Pereira,
  University of Aveiro,
  artur@ua.pt

* Andreia Melo,
  University of Aveiro,
  abmelo@criticalsoftware.com

* Antonio Neves,
  University of Aveiro,
  an@ua.pt

* Joao Figueiredo,
  University of Aveiro
  joao.figueiredo@ieeta.pt

* Miguel Rodrigues,
  University of Aveiro,
  miguel.rodrigues@ua.pt

* Eurico Pedrosa,
  University of Aveiro,
  efp@ua.pt

 Copyright (C) 2001-2022 Universidade de Aveiro
