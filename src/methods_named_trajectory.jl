module MethodsNamedTrajectory

# Does it make sense to have a mutation?
# - Yes for data, No for shapes.

export vec
export get_components
export get_component_names
export add_component!
export remove_component
export remove_components
export update!
export update_bound!
export merge
export get_times
export get_timesteps
export get_duration
export convert_fixed_time
export convert_free_time

export add_suffix
export remove_suffix
export get_suffix

using OrderedCollections
using TestItems

using ..StructNamedTrajectory
using ..StructKnotPoint

# -------------------------------------------------------------- #
# Base show
# -------------------------------------------------------------- #

Base.show(io::IO, traj::NamedTrajectory) = print(io, traj.components, ", T = ", traj.T)

# -------------------------------------------------------------- #
# Base indexing
# -------------------------------------------------------------- #

"""
    KnotPoint(Z::NamedTrajectory, t::Int)

    # Arguments
    - `Z::NamedTrajectory`: The trajectory from which the KnotPoint is taken.
    - `t::Int`: The timestep of the KnotPoint.
"""
function StructKnotPoint.KnotPoint(
    Z::NamedTrajectory,
    t::Int
)
    @assert 1 ≤ t ≤ Z.T
    timestep = get_timesteps(Z)[t]
    return KnotPoint(t, view(Z.data, :, t), timestep, Z.components, Z.names, Z.control_names)
end

"""
    getindex(traj, t::Int)::KnotPoint

Returns the knot point at time `t`.
"""
Base.getindex(traj::NamedTrajectory, t::Int) = KnotPoint(traj, t)

"""
    getindex(traj, ts::AbstractVector{Int})::Vector{KnotPoint}

Returns the knot points at times `ts`.
"""
function Base.getindex(traj::NamedTrajectory, ts::AbstractVector{Int})::Vector{KnotPoint}
    return [traj[t] for t ∈ ts]
end

"""
    lastindex(traj::NamedTrajectory)

Returns the final time index of the trajectory.
"""
Base.lastindex(traj::NamedTrajectory) = traj.T

"""
    getindex(traj, symb::Symbol)

Dispatches indexing of trajectories as either accessing a component or a property via `getproperty`.
"""
Base.getindex(traj::NamedTrajectory, symb::Symbol) = getproperty(traj, symb)

"""
    getproperty(traj, symb::Symbol)

Returns the component of the trajectory with name `symb` (as a view) or the property of the trajectory with name `symb`.
"""
function Base.getproperty(traj::NamedTrajectory, symb::Symbol)
    if symb == :data
        return reshape(view(traj.datavec, :), :, traj.T)
    elseif symb ∈ fieldnames(NamedTrajectory)
        return getfield(traj, symb)
    else
        indices = traj.components[symb]
        return view(traj.data, indices, :)
    end
end

"""
    setproperty!(traj, name::Symbol, val::Any)

Dispatches setting properties of trajectories as either setting a component or a property via `update!` or `setfield!`, respectively.
"""
function Base.setproperty!(traj::NamedTrajectory, symb::Symbol, val::Any)
    if symb ∈ fieldnames(NamedTrajectory)
        setfield!(traj, symb, val)
    else
        update!(traj, symb, val)
    end
end

# -------------------------------------------------------------- #
# Base operations
# -------------------------------------------------------------- #

"""
    vec(::NamedTrajectory)

Returns all variables of the trajectory as a vector, Z⃗.
"""
function Base.vec(Z::NamedTrajectory)
    return vcat(Z.datavec, values(Z.global_data)...)
end

"""
    length(::NamedTrajectory)

Returns the length of all variables of the trajectory, including global data.

TODO: Should global data be in length?
"""
function Base.length(Z::NamedTrajectory)
    return Z.dim * Z.T + Z.global_dim
end

"""
    size(traj::NamedTrajectory) = (dim = traj.dim, T = traj.T)

Returns the size of the trajectory (dim, T), excluding global data.

TODO: Should global data be in size?
"""
Base.size(traj::NamedTrajectory) = (dim = traj.dim, T = traj.T)

"""
    copy(::NamedTrajectory)

Returns a copy of the trajectory.
"""
function Base.copy(traj::NamedTrajectory)
    return NamedTrajectory(deepcopy(traj.data), traj)
end

"""
    isequal(traj1::NamedTrajectory, traj2::NamedTrajectory)

Check if trajectories are equal w.r.t. data using `Base.isequal`
"""
function Base.isequal(traj1::NamedTrajectory, traj2::NamedTrajectory)
    if isequal(traj1.data, traj2.data) &&
        isequal(traj1.names, traj2.names)
        return true
    else
        return false
    end
end

"""
    :(==)(traj1::NamedTrajectory, traj2::NamedTrajectory)

Check if trajectories are equal w.r.t. using `Base.:(==)`
"""
function Base.:(==)(traj1::NamedTrajectory, traj2::NamedTrajectory)
    if traj1.data == traj2.data &&
        traj1.names == traj2.names
        return true
    else
        return false
    end
end

function Base.:*(α::Float64, traj::NamedTrajectory)
    return NamedTrajectory(α * traj.datavec, traj)
end

function Base.:*(traj::NamedTrajectory, α::Float64)
    return NamedTrajectory(α * traj.datavec, traj)
end

function Base.:+(traj1::NamedTrajectory, traj2::NamedTrajectory)
    @assert traj1.names == traj2.names
    @assert traj1.dim == traj2.dim
    @assert traj1.T == traj2.T
    return NamedTrajectory(traj1.datavec + traj2.datavec, traj1)
end

function Base.:-(traj1::NamedTrajectory, traj2::NamedTrajectory)
    @assert traj1.names == traj2.names
    @assert traj1.dim == traj2.dim
    @assert traj1.T == traj2.T
    return NamedTrajectory(traj1.datavec - traj2.datavec, traj1)
end

# -------------------------------------------------------------- #
# Set/get methods
# -------------------------------------------------------------- #

"""
    get_components(::NamedTrajectory)

Returns a NamedTuple containing the names and corresponding data matrices of the trajectory.
"""
function get_components(cnames::Union{Tuple, AbstractVector}, traj::NamedTrajectory)
    symbs = Tuple(c for c in cnames)
    vals = [traj[c] for c ∈ cnames]
    return NamedTuple{symbs}(vals)
end

get_components(traj::NamedTrajectory) = get_components(traj.names, traj)

function filter_by_value(f::Function, nt::NamedTuple)
    return (; (k => v for (k, v) in pairs(nt) if f(v))...)
end

"""
    get_component_names(traj::NamedTrajectory, comps::AbstractVector{<:Int})

Returns the name of the component with the given indices. If only one component is found,
the name is returned as a single symbol. Else, the names are returned as a vector of symbols.

The filter requires that the components are a complete subset of the given indices, so that
a partial match is excluded from the returned names.
"""
function get_component_names(traj::NamedTrajectory, comps::AbstractVector{<:Int})
    name = [n for n ∈ keys(filter_by_value(x -> issubset(x, comps), traj.components)) if n ∈ traj.names]
    if isempty(name)
        error("Component names not found in traj")
    elseif length(name) == 1
        return name[1]
    else
        return name
    end
end

"""
    add_component!(traj, name::Symbol, data::AbstractVecOrMat; type={:state, :control, :slack})

Add a component to the trajectory.

This function resizes the trajectory, so global components and components must be adjusted.
"""
function add_component!(
    traj::NamedTrajectory,
    name::Symbol,
    data::AbstractVecOrMat{Float64};
    type=:state
)

    # check if data is a vector and convert to matrix if so
    if data isa AbstractVector
        data = reshape(data, 1, traj.T)
    end

    # get the dimension of the new component
    dim = size(data, 1)

    # check data against existing data
    @assert size(data, 2) == traj.T
    @assert name ∉ keys(traj.components)
    @assert type ∈ (:state, :control, :slack)


    # update components

    comp_dict = OrderedDict(pairs(traj.components))

    comp_dict[name] = (traj.dim + 1):(traj.dim + dim)

    if type == :state
        comp_dict[:states] = vcat(comp_dict[:states], comp_dict[name])
    elseif type == :control
        comp_dict[:controls] = vcat(comp_dict[:controls], comp_dict[name])
    elseif type == :slack
        if :slacks ∉ keys(comp_dict)
            comp_dict[:slacks] = comp_dict[name]
        else
            comp_dict[:slacks] = vcat(comp_dict[:slacks], comp_dict[name])
        end
    end

    traj.components = NamedTuple(comp_dict)


    # update dims

    traj.dim += dim

    dim_dict = OrderedDict(pairs(traj.dims))

    dim_dict[name] = dim

    if type == :state
        dim_dict[:states] += dim
    elseif type == :control
        dim_dict[:controls] += dim
    elseif type == :slack
        if :slacks ∉ keys(dim_dict)
            dim_dict[:slacks] = dim
        else
            dim_dict[:slacks] += dim
        end
    end

    traj.dims = NamedTuple(dim_dict)


    # update names

    traj.names = (traj.names..., name)
    
    if type == :state
        traj.state_names = (traj.state_names..., name)
    elseif type == :control
        traj.control_names = (traj.control_names..., name)
    elseif type == :slack
        # slack names are states
        traj.state_names = (traj.state_names..., name)
    end

    # update data

    traj.data = vcat(traj.data, data)

    traj.datavec = vec(view(traj.data, :, :))

    # update global data

    global_comps_pairs::Vector{Pair{Symbol, AbstractVector{Int}}} = []
    for (k, v) ∈ pairs(traj.global_components)
        # increase offset for new components
        push!(global_comps_pairs, k => v .+ dim * traj.T)
    end
    traj.global_components = NamedTuple(global_comps_pairs)

    return nothing
end

"""
    remove_component(traj, name::Symbol)

Remove a component from the trajectory.
"""
function remove_component(traj::NamedTrajectory, name::Symbol; kwargs...)
    return remove_components(traj, [name]; kwargs...)
end

"""
    remove_components(traj, names::Vector{Symbol})

Remove a set of components from the trajectory.
"""
function remove_components(
    traj::NamedTrajectory,
    names::AbstractVector{<:Symbol};
    new_timestep::Union{Nothing, Real}=nothing,
    new_control_name::Union{Nothing, Symbol}=nothing,
    new_control_names::Union{Nothing, Tuple{Vararg{Symbol}}}=nothing
)
    @assert all([n ∈ traj.names for n ∈ names])
    @assert isnothing(new_control_name) || isnothing(new_control_names) "Conflicting new control names provided"
    new_control_names = isnothing(new_control_names) ? () : new_control_names
    new_control_names = isnothing(new_control_name) ? (new_control_names...,) : (new_control_name,)
    @assert isnothing(new_control_names) || all([n ∈ traj.names && n ∉ names for n ∈ new_control_names]) "New control names must be valid components"

    comps = NamedTuple([
        (key => data) for (key, data) ∈ pairs(get_components(traj)) if !(key ∈ names)
    ])

    control_names = [n for n ∈ traj.control_names if n ∉ names]
    @assert !isempty(control_names) || !isnothing(new_control_names) "At least one control must be available"
    return NamedTrajectory(
        comps, traj; 
        new_control_names=new_control_names, new_timestep=new_timestep
    )
end

"""
    update!(traj, name::Symbol, data::AbstractMatrix{Float64})

Update a component of the trajectory.
"""
function update!(traj::NamedTrajectory, name::Symbol, data::AbstractMatrix{Float64})
    @assert name ∈ traj.names
    @assert size(data, 1) == traj.dims[name]
    @assert size(data, 2) == traj.T
    traj.data[traj.components[name], :] = data
    return nothing
end

"""
    update!(traj, datavec::AbstractVector{Float64})

Update the trajectory with a new datavec.
"""
function update!(traj::NamedTrajectory, datavec::AbstractVector{Float64})
    @assert length(datavec) == traj.dim * traj.T
    traj.datavec = datavec
    traj.data = reshape(view(datavec, :), traj.dim, traj.T)
    return nothing
end

"""
    update!(traj::NamedTrajectory, data::NamedTuple)

Update the global data of the trajectory with a NamedTuple.
"""
function update!(traj::NamedTrajectory, data::NamedTuple)
    @assert all([k ∈ keys(traj.global_data) for k in keys(data)])
    @assert all([length(v) == traj.global_dims[k] for (k, v) in pairs(data)])
    traj.global_data = merge(traj.global_data, data)
    return nothing
end


"""
    update_bound!(traj, name::Symbol, data::Real)
    update_bound!(traj, name::Symbol, data::AbstractVector{<:Real})
    update_bound!(traj, name::Symbol, data::Tuple{R, R} where R <: Real)

Update the bound of a component of the trajectory.
"""
function update_bound! end

function update_bound!(
    traj::NamedTrajectory,
    name::Symbol,
    new_bound::Real
)
    @assert new_bound > 0 "bound must be positive"
    new_bound = (-fill(new_bound, traj.dims[name]), fill(new_bound, traj.dims[name]))
    update_bound!(traj, name, new_bound)
end

function update_bound!(
    traj::NamedTrajectory,
    name::Symbol,
    new_bound::AbstractVector{<:Real}
)
    @assert all(new_bound .> 0) "bound must be positive"
    new_bound = (-new_bound, new_bound)
    update_bound!(traj, name, new_bound)
end

function update_bound!(
    traj::NamedTrajectory,
    name::Symbol,
    new_bound::Tuple{R, R} where R <: Real
)
    @assert new_bound[1] < new_bound[2] "lower bound must be less than upper bound"
    new_bound = (-fill(new_bound[1], traj.dims[name]), fill(new_bound[2], traj.dims[name]))
    update_bound!(traj, name, new_bound)
end

function update_bound!(traj::NamedTrajectory, name::Symbol, new_bound::BoundType)
    @assert name ∈ keys(traj.components)
    @assert length(new_bound[1]) == length(new_bound[2]) == traj.dims[name]
    if isempty(traj.bounds)
        new_bounds = OrderedDict{Symbol, BoundType}()
    else
        new_bounds = OrderedDict(pairs(traj.bounds))
    end
    new_bounds[name] = new_bound
    new_bounds = NamedTuple(new_bounds)
    traj.bounds = new_bounds
    return nothing
end

# -------------------------------------------------------------- #
# Modify component keys
# -------------------------------------------------------------- #

"""
    add_suffix(obj::T, suffix::String)

Add the suffix to the symbols of the object.
"""
function add_suffix end

add_suffix(symb::Symbol, suffix::String) = Symbol(string(symb, suffix))

function add_suffix(
    symbs::Tuple, 
    suffix::String; 
    exclude::AbstractVector{<:Symbol}=Symbol[]
)
    return Tuple(s ∈ exclude ? s : add_suffix(s, suffix) for s ∈ symbs)
end

function add_suffix(
    symbs::AbstractVector, 
    suffix::String; 
    exclude::AbstractVector{<:Symbol}=Symbol[]
)
    return [s ∈ exclude ? s : add_suffix(s, suffix) for s ∈ symbs]
end

function add_suffix(
    nt::NamedTuple, 
    suffix::String; 
    exclude::AbstractVector{<:Symbol}=Symbol[]
)
    symbs = Tuple(k ∈ exclude ? k : add_suffix(k, suffix) for k ∈ keys(nt))
    return NamedTuple{symbs}(values(nt))
end

function add_suffix(
    components::Union{Tuple, AbstractVector}, 
    traj::NamedTrajectory, 
    suffix::String
)
    return add_suffix(get_components(components, traj), suffix)
end

function add_suffix(traj::NamedTrajectory, suffix::String)
    # Timesteps are appended because of bounds and initial/final constraints.
    component_names = vcat(traj.state_names..., traj.control_names...)
    components = add_suffix(component_names, traj, suffix)
    controls = add_suffix(traj.control_names, suffix)
    return NamedTrajectory(
        components;
        controls=controls,
        timestep=traj.timestep isa Symbol ? add_suffix(traj.timestep, suffix) : traj.timestep,
        bounds=add_suffix(traj.bounds, suffix),
        initial=add_suffix(traj.initial, suffix),
        final=add_suffix(traj.final, suffix),
        goal=add_suffix(traj.goal, suffix)
    )
end


# remove suffix
# -------------

"""
    remove_suffix(obj::T, suffix::String)

Remove the suffix from the symbols of the object.
"""
function remove_suffix end

function remove_suffix(s::String, suffix::String)
    if endswith(s, suffix)
        return chop(s, tail=length(suffix))
    else
        error("Suffix '$suffix' not found at the end of '$s'")
    end
end

remove_suffix(symb::Symbol, suffix::String) = Symbol(remove_suffix(String(symb), suffix))

function remove_suffix(
    symbs::Tuple, 
    suffix::String; 
    exclude::AbstractVector{<:Symbol}=Symbol[]
)
    return Tuple(s ∈ exclude ? s : remove_suffix(s, suffix) for s ∈ symbs)
end

function remove_suffix(
    symbs::AbstractVector,
    suffix::String;
    exclude::AbstractVector{<:Symbol}=Symbol[]
)
    return [s ∈ exclude ? s : remove_suffix(s, suffix) for s ∈ symbs]
end

function remove_suffix(
    nt::NamedTuple, 
    suffix::String; 
    exclude::AbstractVector{<:Symbol}=Symbol[]
)
    symbs = Tuple(k ∈ exclude ? k : remove_suffix(k, suffix) for k ∈ keys(nt))
    return NamedTuple{symbs}(values(nt))
end

# get suffix
# ----------

Base.endswith(symb::Symbol, suffix::AbstractString) = endswith(String(symb), suffix)

"""
    get_suffix(obj::T, suffix::String; remove::Bool=false)

Get the data with the suffix from the object. Remove the suffix if `remove=true`.
"""
function get_suffix end

function get_suffix(
    nt::NamedTuple, 
    suffix::String; 
    remove::Bool=false
)
    names = Tuple(remove ? remove_suffix(k, suffix) : k for (k, v) ∈ pairs(nt) if endswith(k, suffix))
    values = [v for (k, v) ∈ pairs(nt) if endswith(k, suffix)]
    return NamedTuple{names}(values)
end

function get_suffix(
    traj::NamedTrajectory, 
    suffix::String; 
    remove::Bool=false
)
    state_names = Tuple(s for s ∈ traj.state_names if endswith(s, suffix))

    # control names
    if traj.timestep isa Symbol
        if endswith(traj.timestep, suffix)
            control_names = Tuple(s for s ∈ traj.control_names if endswith(s, suffix))
            timestep = remove ? remove_suffix(traj.timestep, suffix) : traj.timestep
            exclude = Symbol[]
        else
            # extract the shared timestep
            control_names = Tuple(s for s ∈ traj.control_names if endswith(s, suffix) || s == traj.timestep)
            timestep = traj.timestep
            exclude = [timestep]
        end
    else
        control_names = Tuple(s for s ∈ traj.control_names if endswith(s, suffix))
        timestep = traj.timestep
        exclude = Symbol[]
    end

    component_names = Tuple(vcat(state_names..., control_names...))
    components = get_components(component_names, traj)
    if remove
        components = remove_suffix(components, suffix; exclude=exclude)
    end

    if isempty(component_names)
        error("No components found with suffix '$suffix'")
    end 

    return NamedTrajectory(
        components,
        controls=remove ? remove_suffix(control_names, suffix; exclude=exclude) : control_names,
        timestep=timestep,
        bounds=get_suffix(traj.bounds, suffix, remove=remove),
        initial=get_suffix(traj.initial, suffix, remove=remove),
        final=get_suffix(traj.final, suffix, remove=remove),
        goal=get_suffix(traj.goal, suffix, remove=remove)
    )
end

# -------------------------------------------------------------- #
# Merge operations
# -------------------------------------------------------------- #

"""
    merge(traj1::NamedTrajectory, traj2::NamedTrajectory)
    merge(trajs::AbstractVector{<:NamedTrajectory})

Returns a new NamedTrajectory object by merging `NamedTrajectory` objects. 

Merge names are used to specify which components to merge by index. If no merge names are provided,
all components are merged and name collisions are not allowed. If merge names are provided, the
names are merged using the data from the index provided in the merge names.

Joined `NamedTrajectory` objects must have the same timestep. If a free time trajectory is desired,
setting the keyword argument `free_time=true` will construct the a component for the timestep.
In this case, the timestep symbol must be provided. 

# Arguments
- `traj1::NamedTrajectory`: The first `NamedTrajectory` object.
- `traj2::NamedTrajectory`: The second `NamedTrajectory` object.
- `free_time::Bool=false`: Whether to construct a free time problem.
- `timestep_name::Symbol=:Δt`: The timestep symbol to use for free time problems.
- `merge_names::Union{Nothing, NamedTuple{<:Any, <:Tuple{Vararg{Int}}}}=nothing`: The names to merge by index.
"""
function Base.merge(traj1::NamedTrajectory, traj2::NamedTrajectory; kwargs...)
    return merge([traj1, traj2]; kwargs...)
end

function Base.merge(
    trajs::AbstractVector{<:NamedTrajectory};
    free_time::Bool=false,
    merge_names::NamedTuple{<:Any, <:Tuple{Vararg{Int}}}=(;),
    timestep_name::Symbol=:Δt,
    timestep_index::Int=timestep_name ∈ keys(merge_names) ? merge_names[timestep_name] : 1
)   
    if length(trajs) < 2
        throw(ArgumentError("At least two trajectories must be provided"))
    end

    # check timestep index 
    if timestep_index < 1 || timestep_index > length(trajs)
        throw(BoundsError(trajs, timestep_index))
    end

    # organize names to drop by trajectory index
    drop_names = map(eachindex(trajs)) do index
        if index < 1 || index > length(trajs) 
            throw(BoundsError(trajs, index))
        end
        Symbol[name for (name, keep) ∈ pairs(merge_names) if keep != index]
    end

    # collect component data
    state_names = Vector{Symbol}[]
    control_names = Vector{Symbol}[]
    for (traj, names) in zip(trajs, drop_names)
        push!(state_names, [s for s ∈ traj.state_names if s ∉ names])
        push!(control_names, [c for c ∈ traj.control_names if c ∉ names])
    end

    # merge states and controls (separately to keep data organized)
    state_components = merge_outer([get_components(s, t) for (s, t) ∈ zip(state_names, trajs)])
    control_components = merge_outer([get_components(c, t) for (c, t) ∈ zip(control_names, trajs)])
    components = merge_outer(state_components, control_components)
    
    # add timesteps (allow a default value, but warn if differences are detected)
    if free_time
        timestep = timestep_name
        if timestep_name ∉ keys(components)
            components = merge_outer(
                components, 
                NamedTuple{(timestep_name,)}([get_timesteps(trajs[timestep_index])])
            )
        end
    else
        timestep = trajs[timestep_index].timestep
        if timestep isa Symbol
            times = get_timesteps(trajs[timestep_index])
            timestep = sum(times) / length(times)
        end
    end

    # check for timestep differences (ignores all free time problems, 
    # as they have key collision if unspecified)
    if timestep_name ∉ keys(merge_names) && !allequal([t.timestep for t in trajs])
        @warn (
            "Automatically merging trajectories with different timesteps.\n" *
            "To avoid this warning, specify the timestep index in merge_names."
        )
    end

    return NamedTrajectory(
        components,
        controls=Tuple(c for c in merge_outer(control_names)),
        timestep=free_time ? timestep_name : timestep,
        bounds=merge_outer(
            [drop(traj.bounds, names) for (traj, names) in zip(trajs, drop_names)]),
        initial=merge_outer(
            [drop(traj.initial, names) for (traj, names) in zip(trajs, drop_names)]),
        final=merge_outer(
            [drop(traj.final, names) for (traj, names) in zip(trajs, drop_names)]),
        goal=merge_outer(
            [drop(traj.goal, names) for (traj, names) in zip(trajs, drop_names)]),
        global_data=merge_outer(
            [drop(traj.global_data, names) for (traj, names) in zip(trajs, drop_names)]),
    )
end

function convert_fixed_time(
    traj::NamedTrajectory;
    timestep_symbol=:Δt,
    timestep = sum(get_timesteps(traj)) / traj.T
)
    @assert timestep_symbol ∈ traj.control_names "Problem must be free time"
    return remove_component(traj, timestep_symbol; new_timestep=timestep)
end

function convert_free_time(
    traj::NamedTrajectory,
    timestep_bounds::BoundType,
    timestep_name=:Δt,
)
    @assert timestep_name ∉ traj.control_names "Problem must not be free time"

    time_bound = (; timestep_name => timestep_bounds,)
    time_data = (; timestep_name => get_timesteps(traj))
    comp_data = get_components(traj)

    return NamedTrajectory(
        merge_outer(comp_data, time_data);
        controls=merge_outer(traj.control_names, (timestep_name,)),
        timestep=timestep_name,
        bounds=merge_outer(traj.bounds, time_bound),
        initial=traj.initial,
        final=traj.final,
        goal=traj.goal
    )
end

function drop(nt::NamedTuple, drop_names::AbstractVector{Symbol})
    if isempty(drop_names)
        return nt
    end
    names = Tuple(k for (k, v) ∈ pairs(nt) if k ∉ drop_names)
    values = [v for (k, v) ∈ pairs(nt) if k ∉ drop_names]
    return NamedTuple{names}(values)
end

drop(nt::NamedTuple, name::Symbol) = drop(nt, [name])

"""
    merge_outer(objs::AbstractVector{<:Any})

Merge objects. An error is reported if a key collision is detected.
"""
function merge_outer(objs::AbstractVector{<:Any})
    return reduce(merge_outer, objs)
end

function merge_outer(objs::AbstractVector{<:Tuple})
    # only construct final tuple
    return Tuple(mᵢ for mᵢ in reduce(merge_outer, [[tᵢ for tᵢ in tup] for tup in objs]))
end

function merge_outer(t1::Tuple, t2::Tuple)
    m = merge_outer([tᵢ for tᵢ in t1], [tⱼ for tⱼ in t2])
    return Tuple(mᵢ for mᵢ in m)
end

function merge_outer(s1::AbstractVector, s2::AbstractVector)
    common_keys = intersect(s1, s2)
    if !isempty(common_keys)
        error("Key collision detected: ", common_keys)
    end
    return vcat(s1, s2)
end

function merge_outer(nt1::NamedTuple, nt2::NamedTuple)
    common_keys = intersect(keys(nt1), keys(nt2))
    if !isempty(common_keys)
        error("Key collision detected: ", common_keys)
    end
    return merge(nt1, nt2)
end

# -------------------------------------------------------------- #
# Time operations
# -------------------------------------------------------------- #

"""
    get_times(traj)::Vector{Float64}

Returns the times of a trajectory as a vector.
"""
function get_times(traj::NamedTrajectory)
    if traj.timestep isa Symbol
        return cumsum([0.0, vec(traj[traj.timestep])[1:end-1]...])
    else
        return [0:traj.T-1...] * traj.timestep
    end
end

"""
    get_timesteps(::NamedTrajectory)

Returns the timesteps of a trajectory as a vector.
"""
function get_timesteps(traj::NamedTrajectory)
    if traj.timestep isa Symbol
        return vec(traj[traj.timestep])
    else
        return fill(traj.timestep, traj.T)
    end
end

"""
    get_duration(::NamedTrajectory)

Returns the duration of a trajectory.
"""
function get_duration(traj::NamedTrajectory)
    return get_times(traj)[end]
end


# =========================================================================== #

@testitem "knot point methods" begin
    include("../test/test_utils.jl")
    fixed_time_traj = get_fixed_time_traj()
    free_time_traj = get_free_time_traj()

    # freetime
    @test free_time_traj[1] isa KnotPoint
    @test free_time_traj[1].x == free_time_traj.x[:, 1]
    @test free_time_traj[end] isa KnotPoint
    @test free_time_traj[end].x == free_time_traj.x[:, end]
    @test free_time_traj[:x] == free_time_traj.x
    @test free_time_traj.timestep isa Symbol

    # fixed time
    @test fixed_time_traj[1] isa KnotPoint
    @test fixed_time_traj[1].x == fixed_time_traj.x[:, 1]
    @test fixed_time_traj[end] isa KnotPoint
    @test fixed_time_traj[end].x == fixed_time_traj.x[:, end]
    @test fixed_time_traj[:x] == fixed_time_traj.x
    @test fixed_time_traj.timestep isa Float64
end

@testitem "algebraic methods" begin
    include("../test/test_utils.jl")
    fixed_time_traj = get_fixed_time_traj()
    free_time_traj = get_free_time_traj()
    free_time_traj2 = copy(free_time_traj)
    fixed_time_traj2 = copy(fixed_time_traj)

    @test (free_time_traj + free_time_traj2).x == free_time_traj.x + free_time_traj2.x
    @test (fixed_time_traj + fixed_time_traj2).x == fixed_time_traj.x + fixed_time_traj2.x

    @test (free_time_traj - free_time_traj2).x == free_time_traj.x - free_time_traj2.x
    @test (fixed_time_traj - fixed_time_traj2).x == fixed_time_traj.x - fixed_time_traj2.x

    @test (2.0 * free_time_traj).x == (free_time_traj * 2.0).x == free_time_traj.x * 2.0
    @test (2.0 * fixed_time_traj).x == (fixed_time_traj * 2.0).x == fixed_time_traj.x * 2.0
end

@testitem "copying and equality checks" begin
    include("../test/test_utils.jl")
    fixed_time_traj = get_fixed_time_traj()
    free_time_traj = get_free_time_traj()

    fixed_time_traj_copy = copy(fixed_time_traj)
    free_time_traj_copy = copy(free_time_traj)

    @test isequal(fixed_time_traj, fixed_time_traj_copy)
    @test fixed_time_traj == fixed_time_traj_copy
end

@testitem "adding and removing state matrix and vector component" begin
    include("../test/test_utils.jl")
    T = 5
    fixed_time_traj = get_fixed_time_traj(T=T)
    free_time_traj = get_free_time_traj(T=T)
    
    # adding state matrix component
    name = :z
    data = rand(2, T)
    type = :state
    
    # case: fixed time
    add_component!(fixed_time_traj, name, data; type=type)
    @test fixed_time_traj.z ≈ data
    @test name ∈ fixed_time_traj.names
    @test name ∈ fixed_time_traj.state_names

    # case: free time
    add_component!(free_time_traj, name, data; type=type)
    @test free_time_traj.z ≈ data
    @test name ∈ free_time_traj.names
    @test name ∈ free_time_traj.state_names

    # adding state vector component
    name = :y
    data = rand(T)
    type = :state

    # case: fixed time
    add_component!(fixed_time_traj, name, data; type=type)
    @test vec(fixed_time_traj.y) ≈ vec(data)
    @test name ∈ fixed_time_traj.names
    @test name ∈ fixed_time_traj.state_names
    
    # case: free time
    add_component!(free_time_traj, name, data; type=type)
    @test vec(free_time_traj.y) ≈ vec(data)
    @test name ∈ free_time_traj.names
    @test name ∈ fixed_time_traj.state_names

    # removing state components
    names = [:z, :y]

    # case: fixed time
    fixed_time_traj = remove_components(fixed_time_traj, names)
    @test all(name ∉ fixed_time_traj.names for name in names)
    @test all(name ∉ fixed_time_traj.state_names for name in names)

    # case: free time
    free_time_traj = remove_components(free_time_traj, names)
    @test all(name ∉ free_time_traj.names for name in names)
    @test all(name ∉ free_time_traj.state_names for name in names)
end

@testitem "adding and removing control matrix component" begin
    include("../test/test_utils.jl")
    T = 5
    fixed_time_traj = get_fixed_time_traj(T=T)
    free_time_traj = get_free_time_traj(T=T)

    # testing adding control component
    name = :a
    data = rand(2, T)
    type = :control

    # case: fixed time
    add_component!(fixed_time_traj, name, data; type=type)
    @test fixed_time_traj.a ≈ data
    @test name ∈ fixed_time_traj.names
    @test name ∈ fixed_time_traj.control_names

    # case: free time
    add_component!(free_time_traj, name, data; type=type)
    @test free_time_traj.a ≈ data
    @test name ∈ free_time_traj.names
    @test name ∈ free_time_traj.control_names

    # testing removing control component
    name = :a

    # case: fixed time
    fixed_time_traj = remove_component(fixed_time_traj, name)
    @test name ∉ fixed_time_traj.names
    @test name ∉ fixed_time_traj.control_names

    # case: free time
    free_time_traj = remove_component(free_time_traj, name)
    @test name ∉ free_time_traj.names
    @test name ∉ free_time_traj.control_names
end

@testitem "adding control vector component" begin
    include("../test/test_utils.jl")
    T = 5
    fixed_time_traj = get_fixed_time_traj(T=T)
    free_time_traj = get_free_time_traj(T=T)

    # testing adding control vector component
    name = :b
    data = rand(T)
    type = :control

    # case: fixed time
    add_component!(fixed_time_traj, name, data; type=type)
    @test vec(fixed_time_traj.b) ≈ vec(data)
    @test name ∈ fixed_time_traj.names
    @test name ∈ fixed_time_traj.control_names

    # case: free time
    add_component!(free_time_traj, name, data; type=type)
    @test vec(free_time_traj.b) ≈ vec(data)
    @test name ∈ free_time_traj.names
    @test name ∈ free_time_traj.control_names
end

@testitem "adding and removing slack matrix component" begin
    include("../test/test_utils.jl")
    T = 5
    fixed_time_traj = get_fixed_time_traj(T=T)
    free_time_traj = get_free_time_traj(T=T)

    # testing adding slack matrix component
    name = :s
    data = rand(2, T)
    type = :slack

    # case: fixed time
    add_component!(fixed_time_traj, name, data; type=type)
    @test fixed_time_traj.s ≈ data
    @test name ∈ fixed_time_traj.names

    # case: free time
    add_component!(free_time_traj, name, data; type=type)
    @test free_time_traj.s ≈ data
    @test name ∈ free_time_traj.names

    # testing removing slack matrix component
    name = :s

    # case: fixed time
    fixed_time_traj = remove_component(fixed_time_traj, name)
    @test name ∉ fixed_time_traj.names
    @test name ∉ fixed_time_traj.state_names

    # case: free time
    free_time_traj = remove_component(free_time_traj, name)
    @test name ∉ free_time_traj.names
    @test name ∉ free_time_traj.state_names
end

@testitem "updating trajectory data" begin
    include("../test/test_utils.jl")
    T = 5
    x_dim = 3
    fixed_time_traj = get_fixed_time_traj(T=T, x_dim=x_dim)
    free_time_traj = get_free_time_traj(T=T, x_dim=x_dim)

    name = :x
    data = rand(x_dim, T)

    # case: fixed time
    update!(fixed_time_traj, name, data)
    @test fixed_time_traj.x == data

    # case: free time
    update!(free_time_traj, name, data)
    @test free_time_traj.x == data
end

@testitem "update all data" begin
    Z = NamedTrajectory((x=rand(2, 5), y=rand(1,5)), controls=:y, timestep=1.0)
    data_original = deepcopy(Z.data)
    datavec_new = rand(length(Z.datavec))
    update!(Z, datavec_new)
    @test Z.datavec == datavec_new
    @test vec(Z.data) == datavec_new
end

@testitem "merge fixed time trajectories" begin
    T = 10
    Δt = 0.1
    traj1 = NamedTrajectory(
        (x = rand(3,T), x1 = rand(2, T), a = rand(2, T)); 
        timestep=Δt, controls=:a
    )
    
    traj2 = NamedTrajectory(
        (x = rand(3,T), x2 = rand(2, T), a = rand(2, T)); 
        timestep=Δt, controls=:a
    )
    
    # Test merge
    traj12 = merge(traj1, traj2, merge_names=(; a=1, x=2))
    @test issetequal(traj12.state_names, (:x, :x1, :x2))
    @test issetequal(traj12.control_names, (:a,))
    @test traj12.x1 == traj1.x1
    @test traj12.x2 == traj2.x2
    @test traj12.a == traj1.a
    @test traj12.x == traj2.x

    traj21 = merge([traj1, traj2], merge_names=(; a=2, x=1))
    @test issetequal(traj21.state_names, (:x, :x1, :x2))
    @test issetequal(traj21.control_names, (:a,))
    @test traj21.x1 == traj1.x1
    @test traj21.x2 == traj2.x2
    @test traj21.a == traj2.a
    @test traj21.x == traj1.x

    # Test collision
    @error merge(traj1, traj2)

    # Test free time
    merge(traj1, traj2, merge_names=(; a=1, x=1), free_time=true).Δt == fill(Δt, T)
end

@testitem "merge many trajectories" begin
    T = 10
    Δt = 0.1
    xs = [Symbol("x$i") for i in 1:5]
    trajs = [
        NamedTrajectory((x => rand(2, T), a = rand(2, T)); timestep=Δt, controls=:a) 
        for x in xs]

    traj = merge(trajs, merge_names=(; a=1))
    @test traj isa NamedTrajectory
    @test issetequal(traj.state_names, xs)
    @test issetequal(traj.control_names, (:a,))
end

@testitem "merge free time trajectories" begin    
    T = 10
    Δt = 0.1
    traj1 = NamedTrajectory(
        (x1 = rand(2, T), a = rand(2, T)); 
        timestep=Δt, controls=:a
    )

    freetraj1 = NamedTrajectory(
        (x1 = rand(2, T), Δt=fill(Δt, T), a = rand(2, T)); 
        timestep=:Δt, controls=(:a, :Δt)
    )
    
    freetraj2 = NamedTrajectory(
        (x2 = rand(2, T), Δt=fill(2Δt, T),  a = rand(2, T)); 
        timestep=:Δt, controls=(:a, :Δt)
    )
    
    traj = merge(freetraj1, freetraj2, merge_names=(; a=1, Δt=1), free_time=true)
    @test traj isa NamedTrajectory
    @test traj.Δt == fill(Δt, (1, T))

    traj = merge(freetraj1, freetraj2, merge_names=(; a=1, Δt=1), free_time=false)
    @test traj isa NamedTrajectory
    @test traj.timestep ≈ Δt

    traj = merge(traj1, freetraj2, merge_names=(; a=1), free_time=true)
    @test traj isa NamedTrajectory
end

@testitem "Free and fixed time conversion" begin
    include("../test/test_utils.jl")

    free_traj = named_trajectory_type_1(free_time=true)
    fixed_traj = named_trajectory_type_1(free_time=false)
    Δt_bounds = free_traj.bounds[:Δt]

    # Test free to fixed time
    @test :Δt ∉ convert_fixed_time(free_traj).control_names

    # Test fixed to free time
    @test :Δt ∈ convert_free_time(fixed_traj, Δt_bounds).control_names

    # Test inverses
    @test convert_free_time(convert_fixed_time(free_traj), Δt_bounds) == free_traj
    @test convert_fixed_time(convert_free_time(fixed_traj, Δt_bounds)) == fixed_traj
end

@testitem "returning times" begin
    include("../test/test_utils.jl")
    T = 5
    fixed_time_traj = get_fixed_time_traj(T=T)
    free_time_traj = get_free_time_traj(T=T)

    # case: free time
    @test get_times(free_time_traj) ≈ [0.0, cumsum(vec(free_time_traj.Δt))[1:end-1]...]

    # case: fixed time
    @test get_times(fixed_time_traj) ≈ 0.1 .* [0:T-1...]
end

@testitem "returning times" begin
    include("../test/test_utils.jl")
    T = 5
    fixed_time_traj = get_fixed_time_traj(T=T)
    free_time_traj = get_free_time_traj(T=T)

    @test size(fixed_time_traj) == (
        dim = sum(fixed_time_traj.dims[fixed_time_traj.names]), T = T
    )
    @test size(free_time_traj) == (
        dim = sum(free_time_traj.dims[free_time_traj.names]), T = T
    )
end

@testitem "Add suffix" begin
    @test add_suffix(:a, "_new") == :a_new

    test = (:a, :b)
    @test add_suffix(test, "_new") == (:a_new, :b_new)
    @test add_suffix(test, "_new", exclude=[:b]) == (:a_new, :b)
    @test add_suffix(test, "_new", exclude=[:a]) == (:a, :b_new)

    test = (a=1, b=2)
    @test add_suffix(test, "_new") == (a_new=1, b_new=2)
    @test add_suffix(test, "_new", exclude=[:b]) == (a_new=1, b=2)
    @test add_suffix(test, "_new", exclude=[:a]) == (a=1, b_new=2)

    test = [:a, :b]
    @test add_suffix(test, "_new") == [:a_new, :b_new]
    @test add_suffix(test, "_new", exclude=[:b]) == [:a_new, :b]
    @test add_suffix(test, "_new", exclude=[:a]) == [:a, :b_new]
end

@testitem "Apply suffix to trajectories" begin
    include("../test/test_utils.jl")

    T = 5
    suffix = "_new"
    fixed_time_traj = get_fixed_time_traj(T=T)
    new_traj = add_suffix(fixed_time_traj, suffix)
    @test new_traj.state_names == add_suffix(fixed_time_traj.state_names, suffix)
    @test new_traj.control_names == add_suffix(fixed_time_traj.control_names, suffix)
    @test fixed_time_traj == add_suffix(fixed_time_traj, "")

    free_time_traj = get_free_time_traj(T=T)
    new_traj = add_suffix(free_time_traj, suffix)
    @test new_traj.state_names == add_suffix(free_time_traj.state_names, suffix)
    @test new_traj.control_names == add_suffix(free_time_traj.control_names, suffix)
    @test free_time_traj == add_suffix(free_time_traj, "")
end

@testitem "Remove suffix" begin 
    @test remove_suffix(:a_new, "_new") == :a
    @error remove_suffix(:a, "_new")

    test = (:a_new, :b_new)
    @test remove_suffix(test, "_new") == (:a, :b)
    @test remove_suffix(test, "_new", exclude=[:b_new]) == (:a, :b_new)
    @test remove_suffix(test, "_new", exclude=[:a_new]) == (:a_new, :b)

    test = (a_new=1, b_new=2)
    @test remove_suffix(test, "_new") == (a=1, b=2)
    @test remove_suffix(test, "_new", exclude=[:b_new]) == (a=1, b_new=2)
    @test remove_suffix(test, "_new", exclude=[:a_new]) == (a_new=1, b=2)

    test = [:a_new, :b_new]
    @test remove_suffix(test, "_new") == [:a, :b]
    @test remove_suffix(test, "_new", exclude=[:b_new]) == [:a, :b_new]
    @test remove_suffix(test, "_new", exclude=[:a_new]) == [:a_new, :b]
end

@testitem "Get suffix" begin
    include("../test/test_utils.jl")

    T = 5
    suffix = "_new"
    fixed_time_traj = get_fixed_time_traj(T=T)
    new_traj = add_suffix(fixed_time_traj, suffix)
    @test get_suffix(new_traj, suffix) == new_traj
    @test get_suffix(new_traj, suffix, remove=true) == fixed_time_traj

    free_time_traj = get_free_time_traj(T=T)
    new_traj = add_suffix(free_time_traj, suffix)
    @test get_suffix(new_traj, suffix) == new_traj
    @test get_suffix(new_traj, suffix, remove=true) == free_time_traj
end

@testitem "Updating trajectory components via view" begin
    traj = rand(NamedTrajectory, 5)

    x_orig = deepcopy(traj.x[:, :])
    u_orig = deepcopy(traj.u[:, :])

    x_new = rand(size(x_orig)...)
    u_new = rand(size(u_orig)...)

    traj.x = deepcopy(x_new)
    @test traj.x == traj.data[traj.components.x, :] == x_new

    traj.u = deepcopy(u_new)
    @test traj.u == traj.data[traj.components.u, :] == u_new

    traj.data[traj.components.x, :] = deepcopy(x_orig)
    @test traj.x == traj.data[traj.components.x, :] == x_orig

    traj.data[traj.components.u, :] = deepcopy(u_orig)
    @test traj.u == traj.data[traj.components.u, :] == u_orig
end

end