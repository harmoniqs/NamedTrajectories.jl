# NamedTrajectories.jl Context

> AI-friendly context for maintaining consistency. Update this when making significant changes.

## Package Purpose

NamedTrajectories.jl provides the **trajectory data structure** for the Piccolo ecosystem. It stores optimization variables as named components in a flattened vector format compatible with NLP solvers.

**Key insight:** The optimizer works with a single flat vector `Z⃗`, but users interact via named components like `traj.x`, `traj[:u]`, `traj[k]`.

## Core Data Structure

```julia
struct NamedTrajectory{R <: Real, ...}
    datavec::AbstractVector{R}    # Flattened trajectory data [z₁; z₂; ...; zₙ]
    N::Int                        # Number of timesteps
    timestep::Symbol              # Name of timestep component (e.g., :Δt)
    dim::Int                      # Total dimension per timestep
    dims::NamedTuple              # Dimension of each component
    bounds::NamedTuple            # Variable bounds (lower, upper)
    initial::NamedTuple           # Initial conditions
    final::NamedTuple             # Final conditions  
    goal::NamedTuple              # Goal states (deprecated, use final)
    components::NamedTuple        # Index ranges for each component
    names::Tuple{Symbol...}       # All component names
    state_names::Tuple{Symbol...} # State component names
    control_names::Tuple{Symbol...} # Control component names
    global_data::AbstractVector{R}  # Global (non-timestep) variables
    global_dim::Int
    global_dims::NamedTuple
    global_components::NamedTuple
    global_names::Tuple{Symbol...}
end
```

## Construction Patterns

### From NamedTuple of matrices
```julia
T = 51  # timesteps
traj = NamedTrajectory(
    (x = rand(4, T), u = rand(2, T), Δt = fill(0.1, T));
    timestep=:Δt,
    controls=:u,                          # or (:u, :du) for multiple
    initial=(x = [1.0, 0.0, 0.0, 0.0],),
    final=(x = [0.0, 1.0, 0.0, 0.0],),
    bounds=(u = (-1.0, 1.0),)             # scalar expands to all dims
)
```

### From existing trajectory (copy with modifications)
```julia
new_traj = NamedTrajectory(old_traj; bounds=(u = (-2.0, 2.0),))
```

### From quantum trajectory (PiccoloQuantumObjects)
```julia
traj = NamedTrajectory(qtraj, N)  # Converts quantum trajectory to optimization form
```

## Access Patterns

```julia
# Matrix view of component (no allocation)
traj.x           # 4×T matrix view
traj[:x]         # Same as traj.x

# KnotPoint at timestep k
kp = traj[k]     # KnotPoint with all components at time k
kp[:x]           # Vector view of x at time k
kp.timestep      # Δt value at time k

# Flattened vector (for optimizer)
Z⃗ = traj.datavec           # Just timestep data
Z⃗_full = vec(traj)         # Including global data (lazy concat)

# Metadata
traj.N                      # Number of timesteps
traj.dim                    # Variables per timestep
traj.components[:x]         # Index range for x (e.g., 1:4)
traj.dims[:x]               # Dimension of x (e.g., 4)
```

## Memory Layout

Data is stored column-major, interleaved by timestep:
```
datavec = [x₁; u₁; Δt₁; x₂; u₂; Δt₂; ... ; xₙ; uₙ; Δtₙ]
           └─ dim ─┘    └─ dim ─┘         └─ dim ─┘
```

Use `slice(k, comps, dim)` from TrajectoryIndexingUtils for correct indexing.

## Modification Methods

```julia
# Update data in-place
update!(traj, name, data_matrix)

# Add new component
new_traj = add_component(traj, :slack, zeros(1, T); type=:control)

# Remove component
new_traj = remove_component(traj, :slack)

# Add derivative components (u → du → ddu)
new_traj = add_control_derivatives(traj, :u, 2)  # Adds :du, :ddu
```

## Time Operations

```julia
get_times(traj)      # Cumulative times [0.0, Δt₁, Δt₁+Δt₂, ...]
get_timesteps(traj)  # Vector of Δt values
get_duration(traj)   # Total duration T
```

## Module Organization

```
NamedTrajectories.jl/src/
├── NamedTrajectories.jl        # Main module (reexports all)
├── struct_named_trajectory.jl  # NamedTrajectory type definition
├── struct_knot_point.jl        # KnotPoint type definition
├── base_named_trajectory.jl    # Base methods (indexing, copy, show)
├── methods_named_trajectory.jl # Operations (add/remove components, etc.)
├── methods_knot_point.jl       # KnotPoint operations
├── utils.jl                    # Helper functions
├── random_trajectories.jl      # Random trajectory generation
└── plotting.jl                 # Makie plotting recipes
```

## Testing

Uses `@testitem` in source files:
```julia
@testitem "NamedTrajectory construction" begin
    # test code
end
```

Run: `julia --project -e 'using Pkg; Pkg.test()'`

## Gotchas

- **timestep is always a control** - Even if not specified, `timestep` symbol is added to `control_names`
- **bounds expand scalars** - `bounds=(u = 1.0,)` becomes `bounds=(u = ([-1.0, -1.0], [1.0, 1.0]),)` for 2D u
- **views not copies** - `traj.x` returns a view; modifications affect `datavec`
- **global_data is separate** - Not part of `datavec`, concatenated lazily in `vec(traj)`
- **`:data` is reserved** - Cannot use as component name
- **KnotPoint is a view** - Modifications to `kp[:x]` affect the trajectory

## Dependencies

- **OrderedCollections.jl** - OrderedDict for component ordering
- **LazyArrays.jl** - Allocation-free vector concatenation
- **Makie.jl** (optional) - Plotting extension
