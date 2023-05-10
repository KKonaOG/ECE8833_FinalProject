# ECE8833 - Computational Intelligence Final Project
## Structure (Project Code in "src" folder)
- Algorithms
    - LN (`Local Navigator: VFH+`)
        - VFHOnPath.m (`Handles VFH Simulation`)
    - PTP (`Point-to-Point: Graft-RRT`)
        - Helpers (`Various Helper Functions for the Graft-RRT`)
        - graft_rrtstar.m (`Graft-RRT Entry Point`)
    - TSP (`Global Navigation: SOM`)
        - Helpers (`Various Helper Functions for the SOM`)
        - self_organizing_map.m (`SOM Entry Point`)
- Digital Twin
    - ObstacleOverride.m (`Implements Obstacle-centric Digital Twin Code`)
    - PriorityOverride.m (`Implements Priority Waypoint-centric Digital Twin Code`)
- Maps
    - Helpers (`Various Helper Functions for Map Setup`)
    - Map1.m
    - Map2.m
    - Map3.m
- Toolboxes
    - MRS Toolbox
- (`Main Entry Point`) main.m


## VFH Parameters
VFH parameters are set directly in the `VFHOnPath.m` file on Lines `40-44`. This was done this way as the map's used in the test scenarios were generally normalized to 16x16, as such similar parameters worked on all maps.

### Modifiable Parameters:
- DistanceLimits: [`lowerLimit` `upperLimit`]
- NumAngularSectors: `scalar`
- HistogramThresholds [`lowerBound` `upperBound`]
- RobotRadius: `scalar`
- SafetyDistance: `scalar`


The VFH is additonally controlled by the configuration of the Pure Pursuit Controller on Lines `34-36`.
### Modifiable Parameters:
- LookaheadDistance: `scalar`
- DesiredLinearVelocity: `scalar`
- MaxAngularVelocity: `scalar`

## LiDAR Parameters
In the event you would like to modify the functionality of the simulated LiDAR, it has a handful of tuneable parameters on Lines `12-14` in `VFHOnPath.m`

### Modifiable Parameters:
- sensorOffset: `[X Offset, Y Offset]`
- scanAngles: `Row Vector of Angular Scan Angles (I.e, linspace(-pi, pi, 180))`
- maxRange: `scalar`

## RRT Parameters
RRT parameters are configured within the individual Map Files typically around lines `13-15`

### Modifiable Parameters:
- map_rrt_epsilon = `scalar`
- map_rrt_iterations = `scalar`
- map_rrt_threshold = `scalar`

## Execution
1. Open `main.m` and navigate to project directory in MATLAB
2. Add the following folders (and their subfolders to MATLAB path):
    *`Main.m does attempt to do this part for you`*
    - src/Algorithms
    - src/Digital Twin
    - src/Maps
    - src/Toolboxes
3. Edit Line 20 from 'run("Maps/Map1.m")' to the desired scenario map
    - Valid Options: Maps/Map1.m, Maps/Map2.m, Maps/Map3.m
4. Set Digital Twin Settings (Only one can be enabled at once `PWT` has priority over `HOT`)
    - Priority Waypoint Digital Twin is enabled by setting the `doPriorityOverride` flag to true
        - `transitionWaypoint` is the waypoint you would like to transition from to the priority waypoint
        - `priorityWaypoint` is the waypoint you would like to be the priority waypoint
    - Hidden Obstacle Digital Twin is enabled by the setting `doObstacleOverride` flag to true
        - `hiddenObstaclePolygon` the obstacle hidden from the initial path planning solution
5. Execute `main.m`