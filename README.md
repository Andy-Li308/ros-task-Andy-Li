# Task Description

Please read through [this](https://www.overleaf.com/read/dmgrrcmpkbkq#211e69) document before moving forward.

### Software Structure
```
- docker -- Where the Dockerfile lives.
- scripts -- Where necessary external scripts live.
- workspace -- Where all the packages live.
- plots -- Where the generated simulation data plots are stored. 
```

### Build the simulator

```bash
./scripts/build/sim.sh
```

### Run the simulator

```bash
./scripts/deploy/devel.sh # To enter the docker container
./scripts/deploy/start.sh # To start the simulation and control task
./scripts/deploy/start.sh --plot # To start the simulation and control task, and generate data plots of the simulation 
ros2 launch limo_simulation limo.launch.py # To launch the simulator
```

When running ./scripts/deploy/start.sh for the first time on the system, you might need to run the following to make app.sh executable: 

```bash
chmod +x ./scripts/deploy/app.sh
```

### What do I edit?

1. Modify the package `limo_control` in the workspace directory for adding your c++ controller program.
2. Make a launch file that can launch everything (Controller and Simulation).
3. Modify `scripts/deploy/app.sh` such that, when `scripts/deploy/start.sh` is run, the task is executed automatically.

### Known Issues

1. This will not work with docker desktop, please do not use it, use the default engine.
2. pandas and matplotlib will need to be installed in order to plot the simulation data. 

Feel free to modify anything else if it does not work as expected.
