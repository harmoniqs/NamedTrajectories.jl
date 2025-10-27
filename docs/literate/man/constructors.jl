# # Constructors

# To construct a `NamedTrajectory` using NamedTrajectories.jl, we simply need to utilize the `NamedTrajectory` constructor.

using NamedTrajectories

## define number of timesteps and timestep
N = 10
dt = 0.1

# build named tuple of components and data matrices.

components = (
    x = rand(3, N),
    u = rand(2, N),
    Δt = fill(dt, 1, N),
)

# we must specify a timestep and control variable for the trajectory.

timestep = :Δt
control = :u

# we can now create a `NamedTrajectory` object.

traj = NamedTrajectory(components; timestep=timestep, controls=control)

# Construct `NamedTrajectory` from previous constructed one.

# `traj = NamedTrajectory(components, traj) # TODO: should this constructor be reimplemented for v0.4.0?`

traj = NamedTrajectory(traj; components=(x=1:3, u=4:5, Δt=6:6))
