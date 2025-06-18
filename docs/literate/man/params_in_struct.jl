# # Parameters
# NamedTrajectory.jl support passing parameters as a Tuple when construct a `NamedTrajectory`.

using NamedTrajectories

# First we need to define number of timesteps and timestep
T = 10
dt = 0.1

# then build named tuple of components and data matrices.
components = (
    x = rand(3, T),
    u = rand(2, T),
    Δt = fill(dt, 1, T),
)

# we must specify a timestep and control variable for the trajectory.

timestep = :Δt
control = :u

# some global params as a NamedTuple
params = (
    α = rand(1),
    β = rand(1)
)

# we can now create a `NamedTrajectory` object with parameters specification.
traj = NamedTrajectory(components, params; timestep=timestep, controls=control)
