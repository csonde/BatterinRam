# BatteringRam

## Description
This software is a tool for planning low speed parking trajectories between 2 points in a map filled with obstacles, while hitting as many objects as possible. Unfortunately it is not very good at the latter.

It uses (almost) Reeds-Shepp Curves [1] and RTT [2].

The software does not do (too much) sanity checks. The algorithm will not detect if the parking is impossible, it will just keep on trying forever. It is assumed that the parking spots are of appropriate size. The algorithm is not supposed to handle extremities, so for example if there are obstacles directly in front of a parking spot, the algorithm may fail even if it is physically possible to execute the parking. Narrow spaces and tight corners may also give the algorithm a run for its money. It is also assumed that the vehicle speed is low enough so that Ackermann-geometry can be applied. Also, if you use right-handed 3D coordinates, you will get a view “from below”.

The algorithm is not optimal in any sense of the word. This is more like a “proof of concept” rather than a “cover all grounds”.

## Requirements
[OpenCV](https://opencv.org/) should be installed on the system. A window handler should also be present through which OpenCV van create windows.

## Usage
~~~
BatteringRam <map_file>
~~~
Every line in the map_file represents an object on the map
The format is the following:
~~~
<id> <p1x> <p1y> <p1z> <p2x>…
~~~
- id can be 0, 1, 2.
- 0 means rectangular parking spot (4 points), entrance is between first and last point.
- 1 means rectangular obstacle (4 points).
- 2 means linear wall (2 points).
- The z coordinate is ignored.
- The Vehicle parameters are hardcoded.

## Controls
- Mouse click – Creates a red blob. Has no effect on the algorithm whatsoever.
- ‘[‘ and ‘]’ – Selects the next or the previous parking spot.
- ‘p’ – Initiates the planning. While planning the only accepted input is ‘p’, which will stop the planning. If the planning is successful, the calculated trajectory will be shown in red. Otherwise, it will just restart after a hardcoded number of iterations.
- ‘r’ – reset the state of the vehicle and the planner.
- ‘s’ – If the planning was successful, the vehicle starts/stops executing the parking in a hardcoded number speed.
- ‘esc’ – close the application

## References

[1]
J. A. Reeds and L. A. Shepp. 
Optimal paths for a car that goes both forwards and backwards. 
Pacific Journal of Mathematics, 145(2):367-393, 1990.

[2]
LaValle, Steven M. (October 1998). "Rapidly-exploring random trees: A new tool for path planning" (PDF). Technical Report. Computer Science Department, Iowa State University (TR 98–11).
