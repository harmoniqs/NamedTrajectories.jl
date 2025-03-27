# ```@meta
# CollapsedDocStrings = true
# ```

#=

# Modifying trajectories

Modifying existing trajectories can be useful for a variety of reasons. Sometimes, you may 
want to change the values of the states, controls, or other components of the trajectory. 
Other times, you may want to add or remove components from the trajectory.

=#

using NamedTrajectories

# Create a random trajectory with 5 time steps, a state variable `x` of dimension 3, and a control variable `u` of dimension 2
traj = rand(NamedTrajectory, 5)
traj.names

# Add a new state variable `y` to the trajectory. Notice this is in-place.
y_data = rand(4, 5)
add_component!(traj, :y, y_data)
traj.names

# Remove the state variable `y` from the trajectory. This is not in place.
restored_traj = remove_component(traj, :y)
restored_traj.names


#=
## Adding suffixes

Another common operation is to add or remove a suffix from the components of a trajectory.
This can be useful when you want to create a modified version of a trajectory that is
related to the original trajectory in some way, or when you want to create a new trajectory
that is a combination of two or more existing trajectories.

For now, these tools are used to create a new trajectory.

=#

# Add a suffix "_new" to the state variable `x`
modified_traj = add_suffix(traj, "_modified")
modified_traj.names

# The modified trajectory contains the same data
modified_traj.x_modified == traj.x

#=
## Merging trajectories

You can also merge two or more trajectories into a single trajectory. This can be useful
when you want to combine data. Mergining trajectories is like taking a direct sum of the
underlying data.

=#

# Merge the original trajectory with the modified trajectory
merged_traj = merge(traj, modified_traj)
merged_traj.names |> println

# You can also extract a specific suffix from the components of a trajectory
extracted_traj = get_suffix(merged_traj, "_modified")
extracted_traj.names

# If you want the original names, you can remove the suffix
original_traj = get_suffix(merged_traj, "_modified", remove=true)
original_traj.names

# ### Merging with conflicts

# If there are any conflicting symbols, you can specify how to resolve the conflict.
conflicting_traj = rand(NamedTrajectory, 5)
traj.names, conflicting_traj.names

# In this case, keep the `u` data from the first trajectory and the `x` data from the second trajectory
merged_traj = merge(traj, conflicting_traj; merge_names=(u=1, x=2,))
println(merged_traj.u == traj.u, ", ", merged_traj.u == conflicting_traj.u)
println(merged_traj.x == traj.x, ", ", merged_traj.x == conflicting_traj.x)

# Merged names
merged_traj.names

#=
## Advanced usage

Sometimes it may be desirable to have direct access to the underlying data matrix/vector associated with the trajectory.
In other circumstances it is more useful to employ the built-in per-component and per-knot-point indexing functionality.
We detail the relationship between these different methods of access here.

=#

traj = rand(NamedTrajectory, 5)

# The "backing store" of a `NamedTrajectory` is its `datavec` field, a `Vector{<:Real}`:

traj.datavec

# The `data` field holds a reshaped "view" of the "backing store", in a form that is somewhat easier to work with:

traj.data

# ### Indexing

# The `data` matrix is of dimension `(traj.dim, traj.T)`, where `length(traj.names) == traj.dim`

# The nth component's indices are given by `traj.components[traj.names[n]]`

println(traj.names)
println(traj.components)

# For instance, the indices of a given component at a given knot point are given as follows:

idx = 1 # x
t = 3
slice = traj.datavec[((t - 1) * traj.T) .+ traj.components[traj.names[idx]]]
println(slice == traj[t].x == traj.x[:, t])

# More generally, the indices of a given component across all knot points are given as follows:

idx = 1 # x
println([((k - 1) * traj.dim) .+ getproperty(traj.components, traj.names[idx]) for k in 1:traj.T])
idx = 2 # u
println([((k - 1) * traj.dim) .+ getproperty(traj.components, traj.names[idx]) for k in 1:traj.T])


# TODO: etc.

# ### Writability

# #### Views and Backing Stores

# In Julia, a "view" (`SubArray`) is intrinsically linked to some "parent" Array. Any in-place modification of one is reflected by the other.

# The following are "safe" operations on a NamedTrajectory (in-place modification of the `datavec`):

traj = rand(NamedTrajectory, 5)
traj.datavec
println(traj.datavec)
traj.datavec[1] *= 0.
println(traj.datavec)
traj.datavec[:] = rand(length(traj.datavec))
println(traj.datavec)

# The following is an example of an "unsafe" operation (non-in-place modification of the `datavec`):

traj = rand(NamedTrajectory, 5)
println(traj.datavec == traj.data[:])
traj.datavec = rand(length(traj.datavec))
println(traj.datavec == traj.data[:]) # the `data` field now points to a "backing store" that is no longer accessible via `traj.datavec`; this will lead to undefined behavior:

# The following is likewise an "unsafe" operation (non-in-place modification of `data`, replacing a "view" of `datavec` with a "raw" matrix):
traj = rand(NamedTrajectory, 5)
println(traj.datavec == traj.data[:])
traj.data = rand(size(traj.data)...)
println(traj.datavec == traj.data[:]) # the `data` field no longer points to any "backing store", i.e. is independent of `traj.datavec`; this will lead to undefined behavior:

# In general, reassigning the values of any of the fields of a trajectory may lead to undefined behavior:
fieldnames(NamedTrajectory)

#=
TODO:
- Prevent this issue by catching attempts to set sensitive fields in `Base.setproperty!(::NamedTrajectory, ::Symbol, ::Any)` (`datavec` and `data` are the primary concern in this regard; however, issuing a warning of some kind may be appropriate).
    - Particularly because it is confusing that `traj.datavec = zeros(length(datavec))` is "discouraged", while `traj.x = zeros(traj.dims.x, traj.T)` and `traj[1].x = zeros(traj.dims.x)` are both valid.
=#

# #### Components and Knot Points

traj = rand(NamedTrajectory, 5)

# Trajectory components are accessible (as a "view") via `getproperty`:

traj.x

# Components are also writable via `setproperty!`:

traj.x = rand(traj.dims.x, traj.T)
traj.x

# or may be modified directly:

traj.x[1] *= 0.
traj.x

# Knot points are likewise accessible via `getindex`:

traj[1]

# A `KnotPoint` behaves much like a `NamedTrajectory`, with respect to getting, setting, and/or modifying its components:

traj[1].u

#

traj[1].u = rand(traj.dims.u)
traj[1].u

#

traj[1].u[1] *= 0
traj[1].u

# The parent trajectory will reflect any modifications made in this fashion:

traj.datavec
