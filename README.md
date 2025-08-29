# AWSIM Labs

This is a fork and extended version of [Autoware Foundation's AWSIM-Labs](https://github.com/autowarefoundation/AWSIM-Labs), supporting some advanced behaviors for NPC vehicles and pedestrians. 

## Additional Features

- Various options to control NPC behaviors, such as, lane change, different acceleration and deceleration profiles for different NPCs, and motion delays.
- A scenario specification language called AWSIM-Script to ease the simulation description.

## Installation

The environment requirements are listed here: https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#environment-preparation.
The setup guide provided by Autoware Foundation is available here: https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/.
Note that this branch (`dev`) may be unstable.

## Using AWSIM-Script and AW-RuntimeMonitor
- AWSIM-Script (https://github.com/duongtd23/AWSIMScript-Client) is a scenario specification language (an example shown below). 
- AW-RuntimeMonitor (https://github.com/duongtd23/AW-RuntimeMonitor) is a runtime monitor that:
  - Records traffic participants' dynamics and ADS (Autoware) internal state (e.g., planning trajectories, control commands, perceived objects, etc.) during simulation and dumps the information to a trace file once the simulation finishes.
  - Can monitor the safety of a control command produced by ADS and if it is unsafe, activate AEB. Check branch [`dev`](https://github.com/duongtd23/AW-RuntimeMonitor/tree/dev) for this feature.

The idea of using AWSIM-Script and AW-RuntimeMonitor together with AWSIM-Labs and Autoware is shown in the figure below. 
We can replace  AWSIM-Script with other scenario description language, e.g., Scenic (interested user can check the [extended Scenic](https://github.com/fomaad/Scenic) to be work with AWSIM-Labs and the corresponding [AW-RuntimeMonitor](https://github.com/duongtd23/AW-RuntimeMonitor/tree/scenic)).

<img src="tool-chain.png" alt="Tool architecture" width="400"/>


### Usage
In addition to AWSIM-Labs (this repo), clone AWSIMScript-Client and AW-RuntimeMonitor (branch `awsimclient`) repos.
1. Launch Autoware and AWSIM-Labs following AWSIM-Labs's documentation.
2. Launch AW-RuntimeMonitor
```bash
cd path-to-autoware
source install/setup.bash
cd AW-RuntimeMonitor
python main.py -o <output-trace-file>
```
Option `-h` might be helpful before running `python main.py`.
Checkout the guide in [AW-RuntimeMonitor](https://github.com/duongtd23/AW-RuntimeMonitor/tree/awsimclient) repo as well.

3. Send the AWSIM-Script input file to be executed.

```bash
cd path-to-autoware
source install/setup.bash
cd AWSIMScript-Client
python client.py <path-to-input-script>
```

Option `-h` might be helpful before running `python client.py`.
An example of the input script is shown below.

4. Once the ego reaches its destination, recorded trace will be saved to `<output-trace-file>`.

### An example of AWSIM-Script
An example of the input accepted by AWSIM-Script is as follows:

```
// Position on lane 226, 20 meters from the start point
spawnPos = "TrafficLane.226" at 20;
goalPos = "TrafficLane.250" at 40;
// The route that NPC will travel
route1 = [
  "TrafficLane.226" max-velocity(10),
  "TrafficLane.427" max-velocity(8.33),
  "TrafficLane.249" max-velocity(8.33),
  // lane change at offset 8m, with longitudinal and lateral velocities 8.33 and 1, respectively
  change-lane(8, 8.33, 1),   
  "TrafficLane.250"
];
npc1 = NPC("smallcar", spawnPos, goalPos, route1, [delay-move-until-ego-engaged(1)]);

route2 = [
  "TrafficLane.248",
  "TrafficLane.449",
  "TrafficLane.264" max-velocity(13.889)
];
npc2 = NPC("taxi", "TrafficLane.248" at 10, "TrafficLane.264", route2, 
  [delay-move-until-ego-move(0), deceleration(9.81), aggressive-driving]);

// A stationary NPC
npc3 = NPC("hatchback", "TrafficLane.249" at 40);

// A pedestrian in elegant style
pedes = Pedes("elegant", [203 # -222.5,  169 # -235], [speed(1.5)]);

// Ego vehicle
ego = Ego("Lexus RX450h 2015 Sample Sensor", spawnPos back 10, "TrafficLane.263" at 60, [max-velocity(8.33)]);

run("Shinjuku", ego, [npc1, npc2, npc3, pedes]);
```

- `npc1` is spawned on lane 226, 20 meters from the start point of the lane. 
It follows three lanes 226, 427, and 249 at speeds of 10 m/s, 8.33 m/s, and 8.33 m/s, respectively.
After traveling 8 meters on lane 249, it starts making a lane change to the adjacent lane 250 with a lateral velocity of 1 m/s. 
It stops on lane 250 after reaching the position which is 40 meters away from the lane's start. 
The last argument of the function `NPC` lets the vehicle delay its movement 1 second after the ego vehicle becomes ready to move.

- `npc2` is a taxi vehicle with aggressive driving behavior, including sudden stops and high deceleration (9.81 m/s2). It starts moving simultaneously with the ego vehicle.

- `npc3` is a stationary hatchback placed on lane 249.

- `pedes` is an elegant-style pedestrian, who goes across the crosswalk with a constant speed of 1.5 m/s.

- `ego` denotes the ego vehicle. Its initial pose is set 10 meters behind npc1 and its goal is on lane 263, 60 meters from the lane’s start point. The script also sets the maximum velocity for the ego at 8.33 m/s.

- The `run` function specifies that the map `Shinjuku` will be used with the participation of the three NPC vehicles and the pedestrian.


### Language Features
- Positions: can be represented by traffic lane and offset or can be relative to other point. For example, `A back 10` would place a vehicle 10 meters behind a given point `A` (in ddition to `back`, the relative keywork can be `forward`, `left`, and `right`).

- NPC Vehicle and pedestrian types: AWSIM-Script supports five vehicle types: taxi, hatchback, van, small car, and truck, along with two pedestrian styles: elegant and casual. 

- Motions for NPCs: For NPC vehicles, their motions can be specified by given a sequence of traffic lanes (and `change-lane`, `cutin`, `cutout`). For NPC pedestrian, their motions should be described through a sequence of (2D) waypoints.

- Ego Configuration: We can set the car model (either "Lexus RX450h 2015 Sample Sensor" or "Lexus RX450h 2015 Sample Sensor Blue"), the initial pose and goal, and the maximum velocity.

- Scenario creation: We can set the map, which is either "Shinjuku" or "ShinjukuNight".

- Other options: a wide array of configuration options, such as, delay spawning/movement, acceleration, and deceleration, can be set for each NPC.
