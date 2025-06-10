# AWSIM Labs

This is a fork and extended version of [Autoware Foundation's AWSIM-Labs](https://github.com/autowarefoundation/AWSIM-Labs), supporting some advanced behaviors for NPC vehicles and pedestrians. 

## Additional Features

- Various options to control NPC behaviors, such as, lane change, different acceleration and deceleration profiles for different NPCs, and motion delays.
- A scenario specification language called AWSIM-Script to ease the simulation description.
- A runtime monitor to record data during simulation. This recorded data can be used to analyze Autoware performance in handling the traffic scenario.

## Installation

The environment requirements are listed here: https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#environment-preparation.

### Steps

1. Clone the repo and checkout branch `v1.3` (this is the stable branch):
```
git clone -b v1.3 https://github.com/duongtd23/AW-Runtime-Verification.git
```

2. Follow the setup guide provided by Autoware Foundation here: https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/.

3. Build the project to export an executable file, assume that its name is `awsimlabs.x86_64`

4. Prepare an input script and Use the following command to run the simulation:
```
./awsimlabs.x86_64 -script <path-to-script-file> -output <path-to-save-traces>
```
where `<path-to-script-file>` and `<path-to-save-traces>` are path to the input script file (a content example is available below) and path to save the trace files, respectively.
Some other available command line arguments are:

- `-noise false`, which will disable noise in lidar data (check details [here](https://github.com/RobotecAI/RobotecGPULidar/blob/develop/docs/GaussianNoise.md)). By default, noise is enabled.

- `-perception_mode camera_lidar_fusion`, which launches the camera-lidar fusion mode. Note that the setup requirement for Autoware (to enable processing camera sensor data) must be done separately. The lidar-only perception mode is launched by default or by replacing `camera_lidar_fusion` by `lidar`.


## AWSIM-Script
### An example
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
