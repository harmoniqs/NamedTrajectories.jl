# # Quickstart Guide

# ## Getting set up

# To install `NamedTrajectories` simply enter the package manager in the Julia REPL with `]` and run

#=
```julia
pkg> add NamedTrajectories
```
=#

# Then just use the package as usual with
using NamedTrajectories

# For the following examples let's work with a simple trajectory

#=
```math
\qty{z_t = \mqty(x_t \\ u_t)}_{t=1:T}
```

where $x_t$ is the state and $u_t$ is the control at a time indexed by $t$. Together $z_t$ is referred to as a *knot point* and a `NamedTrajectory` essentially just stores a collection of knot points and makes it easy to access the state and control variables.

## Creating a variable-timestep `NamedTrajectory`

Here we will create a `NamedTrajectory` with a variable timestep.
=#

## define the number of timesteps
T = 10
Δt = 0.1

## define the knot point data as a NamedTuple of matrices
data = (
    x = rand(3, T),
    u = rand(2, T),
    Δt = fill(Δt, T),
)

## we must specify a timestep and control variable for the NamedTrajectory.
timestep = :Δt
control = :u

## we can now create a `NamedTrajectory` object
traj = NamedTrajectory(data; timestep=timestep, controls=control)

## we can return the names of the stored variables
traj.names


#=
## Adding more problem data

In many settings we will want to specify the problem data of our `NamedTrajectory` -- e.g. bounds, initial values, final values, and goal values. 
=#

## define the number of timesteps
T = 10

## define the knot point data as a NamedTuple of matrices
data = (
    x = rand(3, T),
    u = rand(2, T),
    Δt = rand(T),
)

## define initial values
initial = (
    x = [1.0, 0.0, 0.0],
    u = [0.0, 0.0],
)

## define final value, here just on the control
final = (
    u = [0.0, 0.0],
)

## define bounds
bounds = (
    x = 1.0,
    u = 1.0
)

## set a goal for the state
goal = (
    x = [0.0, 0.0, 1.0],
)

## we must specify a timestep and control variable for the NamedTrajectory
timestep = :Δt
control = :u

## we can now create a `NamedTrajectory` object
traj = NamedTrajectory(
    data;
    timestep=timestep,
    controls=control,
    initial=initial,
    final=final,
    bounds=bounds,
    goal=goal
)

## we can then show the bounds
traj.bounds


# ## Retrieving data

# There are a number of ways to access data, for example

traj.x

# returns the data matrix associated with the state variable `x`.

traj.data

# returns the all of the data as a matrix where each column is a knot point.

traj.datavec

# returns the all of the data as a view of the data matrix as a vector -- useful for passing data to solvers.

traj[1]

# returns a `KnotPoint`.

traj[1].x

# returns the state at the first knot point.

get_times(traj)

# returns the times of the knot points.

get_timesteps(traj)

# returns the timesteps of the knot points, as vector.


# ## Retrieving metadata

# We can also retrieve metadata about the trajectory, for example

traj.names

# returns the names of the variables stored in the trajectory.

traj.dims

# returns the dimensions of the variables stored in the trajectory.

traj.T

# returns the number of knot points in the trajectory.

traj.components

# returns the components of the trajectory.

#=
## Updating problem data

The `NamedTrajectory` can be updated by accessing fields and replacing the data.


We also have `update!` to update trajectory components, and  `update_bound!`, which allows you to pass in the same kinds of bounds available at construction (e.g., an `Int` or `Tuple`). The bound will get shaped to match the trajectory component dimensions just like at construction.These methods cannot be used to update non-existent bounds or components.

For efficiency, a trajectory cannot add new data after it is constructed. However, we have convenience methods like `add_component` that build a new trajectory with added data.

=#

update_bound!(traj, :x, )