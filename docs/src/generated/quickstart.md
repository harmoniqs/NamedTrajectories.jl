```@meta
EditURL = "../../literate/quickstart.jl"
```

# Quickstart Guide

## Getting set up

To install `NamedTrajectories` simply enter the package manager in the Julia REPL with `]` and run

```julia
pkg> add NamedTrajectories
```

Then just use the package as usual with

````@example quickstart
using NamedTrajectories
````

For the following examples let's work with a simple trajectory

```math
\qty{z_t = \mqty(x_t \\ u_t)}_{t=1:T}
```

where $x_t$ is the state and $u_t$ is the control at a time indexed by $t$. Together $z_t$ is referred to as a *knot point* and a `NamedTrajectory` essentially just stores a collection of knot points and makes it easy to access the state and control variables.

## Creating a fixed-timestep `NamedTrajectory`

Here we will createa a `NamedTrajectory` with a fixed timestep. This is done by passing a scalar as the `timestep` kwarg.

````@example quickstart
# define the number of timesteps
T = 10

# define the knot point data as a NamedTuple of matrices
data = (
    x = rand(3, T),
    u = rand(2, T),
)

# we must specify a timestep and control variable for the trajectory
timestep = 0.1
control = :u

# we can now create a `NamedTrajectory` object
traj = NamedTrajectory(data; timestep=timestep, controls=control)

# we can return the names of the stored variables
traj.names
````

Let's plot this trajectory

````@example quickstart
plot(traj)
````

## Creating a variable-timestep `NamedTrajectory`

Here we will create a `NamedTrajectory` with a variable timestep. This is done by passing a `Symbol`, corresponding to component of the data, as the `timestep` kwarg.

````@example quickstart
# define the number of timesteps
T = 10

# define the knot point data as a NamedTuple of matrices
data = (
    x = rand(3, T),
    u = rand(2, T),
    Δt = rand(T),
)

# we must specify a timestep and control variable for the NamedTrajectory
timestep = :Δt
control = :u

# we can now create a `NamedTrajectory` object
traj = NamedTrajectory(data; timestep=timestep, controls=control)

# we can return the names of the stored variables
traj.names
````

## Adding more problem data

In many settings we will want to add problem data to our `NamedTrajectory` -- e.g. bounds, initial values, final values, and goal values. This is realized by passing NamedTuples containing this data.

````@example quickstart
# define the number of timesteps
T = 10

# define the knot point data as a NamedTuple of matrices
data = (
    x = rand(3, T),
    u = rand(2, T),
    Δt = rand(T),
)

# define initial values
initial = (
    x = [1.0, 0.0, 0.0],
    u = [0.0, 0.0],
)

# define final value, here just on the control
final = (
    u = [0.0, 0.0],
)

# define bounds
bounds = (
    x = 1.0,
    u = 1.0
)

# set a goal for the state
goal = (
    x = [0.0, 0.0, 1.0],
)

# we must specify a timestep and control variable for the NamedTrajectory
timestep = :Δt
control = :u

# we can now create a `NamedTrajectory` object
traj = NamedTrajectory(
    data;
    timestep=timestep,
    controls=control,
    initial=initial,
    final=final,
    bounds=bounds,
    goal=goal
)

# we can then show the bounds
traj.goal
````

## Retrieving data

There are a number of ways to access data, for example

````@example quickstart
traj.x
````

returns the data matrix associated with the state variable `x`.

````@example quickstart
traj.data
````

returns the all of the data as a matrix where each column is a knot point.

````@example quickstart
traj.datavec
````

returns the all of the data as a view of the data matrix as a vector -- useful for passing data to solvers.

````@example quickstart
traj[1]
````

returns a `KnotPoint`.

````@example quickstart
traj[1].x
````

returns the state at the first knot point.

````@example quickstart
get_times(traj)
````

returns the times of the knot points.

````@example quickstart
get_timesteps(traj)
````

returns the timesteps of the knot points, as vector.

## Retrieving metadata

We can also retrieve metadata about the trajectory, for example

````@example quickstart
traj.names
````

returns the names of the variables stored in the trajectory.

````@example quickstart
traj.dims
````

returns the dimensions of the variables stored in the trajectory.

````@example quickstart
traj.T
````

returns the number of knot points in the trajectory.

````@example quickstart
traj.components
````

returns the components of the trajectory.

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

