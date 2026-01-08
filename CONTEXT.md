# NamedTrajectories.jl Context

> AI-friendly context for maintaining consistency. Update this when making significant changes.

## Package Purpose

NamedTrajectories.jl provides a **data structure for time-series data with named components**. It abstracts away messy indexing and vectorization required for numerical optimization while providing intuitive named access.

**Key insight:** Optimizers need flat vectors, but humans need named structure. NamedTrajectory bridges this gap.

## Core Abstractions

### NamedTrajectory

A trajectory with named components (states, controls, timesteps):

```julia
traj = NamedTrajectory(
    (x = rand(2, T), u = rand(1, T), Δt = fill(0.1, T));
    timestep=:Δt,           # Which component is the timestep
    controls=:u,            # Which components are controls
    initial=(x = [0.0, 0.0],),  # Initial conditions
    final=(x = [1.0, 0.0],),    # Final conditions
    bounds=(u = (-1.0, 1.0),)   # Variable bounds
)
```

### Key Properties

```julia
# Dimensions
traj.T           # Number of timesteps
traj.dim         # Total variables per timestep
traj.dims        # Dict mapping name → size

# Components
traj.components  # Dict mapping name → index range
traj.names       # All component names

# Data access
traj.x           # Matrix (size × T) - by name
traj[:x]         # Same as above
traj[k]          # KnotPoint at timestep k
traj[end]        # Last KnotPoint
traj.datavec     # Flattened vector (for optimizer)

# Metadata
traj.timestep    # Symbol for timestep component (:Δt)
traj.controls    # Tuple of control symbols ((:u,))
traj.initial     # NamedTuple of initial conditions
traj.final       # NamedTuple of final conditions
traj.bounds      # NamedTuple of bounds
traj.goal        # Optional goal data
```

### KnotPoint

A single timestep of data:

```julia
kp = traj[k]     # Get KnotPoint at timestep k

# Access components
kp.x             # Vector of state at time k
kp[:x]           # Same as above
kp.u             # Vector of controls at time k

# Get all data as vector
vec(kp)          # Flattened data at this timestep
```

## Common Operations

### Creating Trajectories

```julia
# From NamedTuple of matrices
data = (x = rand(4, 100), u = rand(2, 100), Δt = fill(0.01, 100))
traj = NamedTrajectory(data; timestep=:Δt, controls=:u)

# With bounds
traj = NamedTrajectory(data;
    timestep=:Δt,
    controls=:u,
    bounds=(u = [(-1.0, 1.0), (-0.5, 0.5)],)  # Per-component bounds
)

# With initial/final conditions
traj = NamedTrajectory(data;
    timestep=:Δt,
    initial=(x = zeros(4),),
    final=(x = ones(4),)
)
```

### Accessing Data

```julia
# By name - returns matrix
X = traj.x           # 4 × T matrix
U = traj.u           # 2 × T matrix

# By timestep - returns KnotPoint
kp = traj[1]         # First timestep
kp = traj[end]       # Last timestep

# Component at timestep
x_at_5 = traj.x[:, 5]      # Via matrix
x_at_5 = traj[5].x         # Via KnotPoint

# Flattened vector (for optimizer)
z = traj.datavec     # Vector of length dim × T
```

### Modifying Trajectories

```julia
# Direct assignment (matrices)
traj.u .= new_controls

# Via datavec (for optimization)
z = traj.datavec
z .= optimized_values  # Modifies traj in-place

# Copy trajectory
new_traj = copy(traj)
```

### Getting Time Information

```julia
# Timesteps
Δts = get_timesteps(traj)    # Vector of Δt values
T_total = sum(Δts)           # Total duration

# Accumulated time
times = get_times(traj)      # [0, Δt₁, Δt₁+Δt₂, ...]
```

### Indexing Helpers

```julia
using TrajectoryIndexingUtils: slice

# Get indices into flattened vector
dim = traj.dim
x_comps = traj.components[:x]

# All variables at timestep k
indices_k = slice(k, dim)

# Specific component at timestep k
x_indices_k = slice(k, x_comps, dim)
```

## Bounds and Constraints

### Specifying Bounds

```julia
# Scalar bounds (same for all components)
bounds=(u = (-1.0, 1.0),)

# Per-component bounds (vector of tuples)
bounds=(u = [(-1.0, 1.0), (-0.5, 0.5)],)

# Asymmetric bounds
bounds=(x = [(-Inf, 10.0), (0.0, Inf)],)
```

### Initial/Final Conditions

```julia
# Full state constraints
initial=(x = zeros(4),)
final=(x = ones(4),)

# Partial constraints (only some components)
initial=(x = [0.0, missing, missing, 0.0],)  # Only constrain x[1] and x[4]
```

## Integration with DirectTrajOpt

NamedTrajectory provides the data structure that DirectTrajOpt optimizes:

```julia
# DirectTrajOpt extracts constraints from trajectory metadata
constraints = get_trajectory_constraints(traj)
# Returns EqualityConstraints for initial/final, BoundsConstraints for bounds

# During optimization, the datavec is what changes
z = traj.datavec
# Optimizer modifies z, which modifies traj in-place

# After solve, access results by name
optimal_controls = traj.u
optimal_states = traj.x
```

## Integration with QuantumCollocation

Quantum trajectories convert to NamedTrajectory for optimization:

```julia
# From quantum trajectory
qtraj = UnitaryTrajectory(sys, pulse, U_goal)
traj = NamedTrajectory(qtraj, N)  # N = number of optimization knot points

# Now has components like:
# :Ũ⃗  - isomorphism-vectorized unitary (2n² reals)
# :u  - controls
# :Δt - timesteps
# :t  - accumulated time (if time-dependent)
```

## Plotting

```julia
using PiccoloPlots  # or NamedTrajectories.plot

# Plot specific components
plot(traj, [:x, :u])

# Plot all components
plot(traj)

# Customize
plot(traj, [:u]; 
    title="Controls",
    xlabel="Time",
    ylabel="Amplitude"
)
```

## Module Structure

```
NamedTrajectories.jl/src/
├── NamedTrajectories.jl       # Main module
├── struct_named_trajectory.jl # NamedTrajectory struct
├── struct_knot_point.jl       # KnotPoint struct
├── methods_named_trajectory.jl # NamedTrajectory methods
├── methods_knot_point.jl      # KnotPoint methods
├── plotting.jl                # Plot recipes
├── random_trajectories.jl     # Test utilities
└── utils.jl                   # Helper functions
```

## Gotchas

- `traj.datavec` is a **view** - modifying it modifies the trajectory
- Bounds apply to **all timesteps** (or specify time indices if needed)
- Initial/final conditions become equality constraints in optimization
- Component indices in `traj.components` are 1-based ranges
- Use `slice()` for converting between component-time indexing and flat indices
- `copy(traj)` creates a deep copy with independent data
