## 2D Particle Filter

A 2 dimensional particle filter in C++. Takes as input a map and some initial localization information (analogous to what a GPS would provide) and at each time step the filter receives (noisy) sensor and control data.

[![Vehicle Tracking Particle Filter](http://img.youtube.com/vi/SI6gERmsuZ4/0.jpg)](https://youtu.be/SI6gERmsuZ4 "Vehicle Tracking Particle Filter")

## Running the Code

```
> ./clean.sh
> ./build.sh
> ./run.sh
```

### Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
* On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./particle_filter`

### Build Instructions (xcode)

1. Clone this repo.
2. Make a build directory: `mkdir xcode-build && cd xcode-build`
3. Run: `cmake -G Xcode ../`
4. Open `PARTICLE_FILTER.xcodeproj` in xcode

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

> * Map data provided by 3D Mapping Solutions GmbH.


#### Control Data
`control_data.txt` contains rows of control data. Each row corresponds to the control data for the corresponding time step. The two columns represent
1. vehicle speed (in meters per second)
2. vehicle yaw rate (in radians per second)

#### Observation Data
The `observation` directory includes around 2000 files. Each file is numbered according to the timestep in which that observation takes place.

These files contain observation data for all "observable" landmarks. Here observable means the landmark is sufficiently close to the vehicle. Each row in these files corresponds to a single landmark. The two columns represent:
1. x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE.
2. y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE.

> **NOTE**
> The vehicle's coordinate system is NOT the map coordinate system.
