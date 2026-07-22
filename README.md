# OrbitCore

OrbitCore is a small C++17 spacecraft navigation simulator focused on vector math and orbital physics calculations.

It calculates:

- 3D spacecraft position and velocity
- Distance between a spacecraft and a target
- Newtonian gravitational force
- Circular orbital velocity
- Acceleration caused by gravity
- Estimated future position using time steps
- Fuel usage during a maneuver
- Escape velocity, orbital period, orbital energy, kinetic energy, and potential energy
- Hohmann transfer delta-v and transfer time between circular orbits
- Rocket equation delta-v from specific impulse and mass ratio
- Thrust acceleration
- Direction vectors toward a target
- Dot product, cross product, vector magnitude, and normalization
- Collision risk when spacecraft move too close together
- Space base distance, docking range, and approach direction

## Build and run

```sh
cmake -S . -B build
cmake --build build
./build/orbitcore_sim
```

On Windows with MinGW:

```sh
cmake -S . -B build-mingw -G "MinGW Makefiles"
cmake --build build-mingw
./build-mingw/orbitcore_sim.exe
```

With Visual Studio generators, the executable is usually under `build/Debug/orbitcore_sim.exe`.

## Run tests

```sh
cmake --build build
ctest --test-dir build --output-on-failure
```

## Project layout

- `include/OrbitCore/Vector3.hpp` - 3D vector operations
- `include/OrbitCore/Spacecraft.hpp` - spacecraft state and fuel accounting
- `include/OrbitCore/SpaceBase.hpp` - orbital station/base approach checks
- `include/OrbitCore/Physics.hpp` - orbital mechanics and navigation functions
- `src/main.cpp` - simple simulation loop
- `tests/OrbitCoreTests.cpp` - fast no-framework checks
