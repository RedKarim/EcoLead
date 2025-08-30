# EcoLead Simulation Usage

## Running the Simulation

The EcoLead simulation now supports two different modes via command line arguments:

### MPC Mode (Original)

```bash
cd src
python3 main.py MPC
```

**Features:**

- Full platoon simulation with leader and follower vehicles
- MPC (Model Predictive Control) for ego vehicle
- SPaT-driven velocity optimization
- Dynamic platoon splitting at traffic lights
- V2X communication simulation (PAM/PCM messages)
- Up to 7 following vehicles with 36m spacing

### IDM Mode (New)

```bash
cd src
python3 main.py IDM
```

**Features:**

- IDM ego vehicle + following vehicles simulation
- IDM (Intelligent Driver Model) control logic for ego vehicle
- CACC/PID control for following vehicles (same as MPC mode)
- Traffic light awareness and stopping behavior for ego vehicle
- Platoon formation with IDM leader and CACC followers

## Key Differences

| Feature           | MPC Mode                      | IDM Mode                           |
| ----------------- | ----------------------------- | ---------------------------------- |
| Controller        | Model Predictive Control      | IDM (ego) + CACC/PID (followers)   |
| Vehicles          | Ego + 7 followers             | Ego + 7 followers                  |
| Optimization      | Fuel consumption + trajectory | Car following + traffic rules      |
| Complexity        | High (50-step prediction)     | Medium (reactive + following)      |
| V2X Messages      | Yes (PAM/PCM)                 | Yes (PAM/PCM for followers)        |
| Platoon Splitting | Yes                           | No (simplified platoon management) |

## Traffic Light Behavior

### MPC Mode

- Uses SPaT data to optimize velocity profiles
- Calculates feasible green windows
- Coordinates entire platoon through intersections

### IDM Mode

- Detects red/yellow traffic lights ahead
- Applies IDM logic to stop at traffic lights
- Uses position-based stopping similar to MATLAB reference

## Configuration

Both modes use the same:

- Route waypoints (`config/route.csv`)
- Traffic light settings (in `main.py`)
- CARLA world settings (synchronous mode, 10 FPS)

MPC-specific configuration in `config/config.yaml` is only used in MPC mode.

## Output

Both modes generate CSV files with simulation data for analysis using `evaluation/evaluate.ipynb`.
