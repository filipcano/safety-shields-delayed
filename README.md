# Safety Shielding under Delayed Observation


https://github.com/filipcano/safety-shields-delayed/assets/109427334/b230a7c5-fb15-4b42-ba44-1a373a9e7aac


## Requirements
The first set of the experiments can be executed with the VM as default with 8GB of RAM.
For the second set of experiments, both the shield computation and the running of the CARLA simulation  
the resources required to generate the shields are similar, but to run the CARLA simulator properly the VM requires much more resources.
We have not been able to fully run the car simulations with under 16GB of RAM,
and even with 16GB the performance is poor in a VM.
More memory and access to a GPU are advisable.

## Folders
### src
Contains the main files.

#### delayed_shields.h & delayed_shields.cpp
The class and header file containing the objects and algorithms needed to
compute shields resilient to delayed inputs.
This class is used for all experiments. 


#### chase_game.h & chase_game.cpp
The class and header file to run the chase game experiment.

#### main_chase.cpp
Script that reads input files from the chase game,
as well as arguments,
and executes the previous class according to such input.

#### pedestrian_shield.cpp
Implements the car to pedestrian intersection shield.

#### car_shield.cpp
Implements the car to car intersection shield.

#### compile.sh
Compiles the previous files and generates the executables to reproduce the experiments.

#### inputs_chase_game
Folder that contains inputs for the chase game experiments.
In particular, this folder contains the input files park1.txt, ... , park7.txt,
with the scenarios described in figure 6.

#### chase_game_experiments
Experiment scripts to reproduce results in figures 7 and 8.

#### carla_experiments
Experiment scripts to reproduce results in figure 9.
The contents of this folder are:
##### traffic_scenarios.py
This file allows you to run the specified scenario and shield.  

##### plot_experiment.py
This file allows you to plot the observations from `traffic_scenarios.py`

##### parse_shield.py
This file allows you to parse the shield `.txt` file and save it either compressed or as a pickle file. The shield file can then be loaded in `traffic_scenarios.py`.

##### process_shield_file.py
This file is a wrapper of the previous one,
that can process a '.txt' file for many delays.

To see a full argument description of the python files use:  

    python file_name.py --help

##### experiments_data
This folder contains the data we gathered and used to produce the graphs in figure 9.

##### shield
This is a container folder for the shield files generated by parse_shield.py and to be used by traffic_scenarios.py.


## Use instructions
### Chase Game
The executable main_chase.o expects an input (see an example in src/inputs_chase_game/ex1.txt) consisting of the width and height of the grid, followed by the number of obstacles N_o.
Then follow N_o lines, each indicating the coordinates of an obstacle.
Then follows a line with the number of kid's moves N_k,
followed by a line containing N_k strings representing the kid's moves.
Then follows a line with the number of robot's moves N_k,
followed by a line containing N_k strings representing the robot's moves.
See any of the files in the inputs_chase_game folder as an example.

Once the input has been read, the corresponding class of chaseGame is generated,
and the corresponding shield is computed with an instance of the delayedShield class.

After the generation of the gridworld and the shield,
a round of the chase game is played for 1000 steps,
and printed in console.

The user can run an example with:

    ./main_chase.o <inputs_chase_game/ex1.txt

The executable main_chase.o can take the following arguments:

- "delta=d", where "d" is the maximum delay for which the shield is computed. It is also the delay that the robot has while playing. It is set to 0 by default.
- "steps_of_each_game=N_s" is the number of steps each round of the game is played. Default number is 1000.
- "number_of_rounds=N" is the number of rounds of the game played. If N > 0, then N rounds are played, and aggregated statistics of score and number of shield interventions are printed in the end of the last round.
If N = 0, one round is played. The default value is 0.
To actually not play any round, the following command is used.
- "dont_play" indicates the program to just compute the shield,
but not play any round. Default is false.
- "seed=s", where "s" is the random seed used. Default is 1.
- "play_controllable" indicates the program to compute the post-shield maximizing the delay-resilient level, and plays N_r rounds with such strategy. Default is false.
- "play_safest" indicates the program to compute the post-shield maximizing the robust-safety level, and plays N_r rounds with such strategy. Default is false.

#### Reproduce experiments
To reproduce the experiments in figure 7 and 8, the scripts exp1, exp2 and exp3 in the folder src/chase_game_experiments are used.

For each experiment, there is a bash script that runs the experiment and dumps the results in a txt file. This txt file is read by a python script, that generates the corresponding graph. The whole pipeline for exp1 is:

    bash exp1.sh
    python3 exp1.py

The pipeline for exp2 and exp3 is the same.
We have included the original data, so that the generation of the graph can be done independently from the generation of the data.

Unfortunately, the datapoints in figure 7 regarding the implementation by Chen et. al. require the software Mathematica, that cannot run in this VM without a license. We have included the values we obtained in our experiments in the python file for the sake of completeness.

### Carla experiments

The overview is the following:
To generate the shield file, execute pedestrian_shield.o or car_shield.o,
depending on the scenario to run.
This file is the processed with the script process_shield_file.py,
that generates shield file for each delay in pickle format.
These files are then given as an input to the main script traffic_scenarios.py,
that generates the scenarios, runs the car simulation and stores the data of the simulation. The data is then process with plot_experiment.py to generate the graphs in figure 9.

##### Scenarios
1. crossing with a frontal crash of the ego vehicle
4. pedestrians at a cross-walk

Scenarios 2 and 3 are deprecated.

#### Pipeline for car to car scenario
Generate the shield and save it in the shields folder:

    ./car_shield.o >shield/car_shield.txt

    Process the shield .txt file into pickle files.
    Each delay computed requires a separate shield file.
    This is done executing

    python3 process_shield_file -f shield/car_shield

To run the scenario in Carla, first you need to have Carla running. To do so, go to dependencies/CARLA_0.9.11/ and in a terminal execute:

    ./CarlaUE4.sh -quality-level=Low -opengl

This command launches a Carla simulation with Low quality rendering and opengl graphics.
We recommend this setting. If the VM is running with enough resources, you can switch to -quality-level=Epic.
The option of not using OpenGL but Vulkan is available in Carla,
but this artifact does not support it,
because we wanted to minimize the setup process.

Once a Carla simulation is running, go to src/carla_experiments/ and execute, for example:

    python3 traffic_scenarios.py --scenario 1 --delay 0 --shield shield/car_shield_d0.pickle --display_shield --save-obs car_shield_d0.csv

This executes scenario 1 with delay 0, and saves the observations in a file car_shield_d0.csv.
To run other delays, you need to change the --delay option, and also you need to point to the appropriate shield file.
There is one shield pickle file for each delay.
Please bear in mind that the delay count is different in the Carla experiments.
The shields are named with the delay of player 0 only,
so what in the paper (and in the figures) is marked as delay 0, 2, 4, 6;
in traffic_scenarios.py corresponds to delays 0, 1, 2, 3.

Once the csv files are generated, place them in the src/carla_experiments/experiment/ folder,
following the naming of the provided datafiles, that is crossing_d{delay}.csv for the car to car scenario,
and pedestrian_d{delay}.csv for the car to pedestrian scenario.

Then run

    python3 plot_experiment.py

#### Pipeline for car to pedestrian scenario
The pipeline for the car to pedestrian intersection is very similar to the previous one.

Generate the shield and save it in the shields folder:

    ./pedestrian_shield.o >shield/pedestrian_shield.txt

Process the shield .txt file into pickle files.
Each delay computed requires a separate shield file.
This is done executing

    python3 process_shield_file -f shield/pedestrian_shield

To run the scenario in Carla, first you need to have Carla running. To do so, go to dependencies/CARLA_0.9.11/ and in a terminal execute:

    ./CarlaUE4.sh -quality-level=Low -opengl

Once a Carla simulation is running, go to src/carla_experiments/ and execute, for example:

    python3 traffic_scenarios.py --scenario 4 --delay 1 --shield shield/pedestrian_shield_1.pickle --display_shield --save-obs pedestrian_shield_d1.csv

This executes scenario 4 with delay 1, and saves the observations in a file pedestrian_shield_d1.csv.

Once the csv files are generated, place them in the src/carla_experiments/experiment/ folder,
following the naming of the provided datafiles, that is crossing_d{delay}.csv for the car to car scenario,
and pedestrian_d{delay}.csv for the car to pedestrian scenario.

Then run

    python3 plot_experiment.py
