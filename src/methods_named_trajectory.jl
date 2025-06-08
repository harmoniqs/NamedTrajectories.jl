module MethodsNamedTrajectory

export get_components
export get_component_name
export get_component_names

export get_times
export get_timesteps
export get_duration

export add_component
export add_components

export remove_component
export remove_components

export update!


# ---

# TODO: Refactor 

export update_bound!

export merge
export add_suffix
export remove_suffix
export get_suffix

# TOOD: remove?
# export convert_fixed_time
# export convert_free_time

# ---

using OrderedCollections
using TestItems

using ..StructNamedTrajectory
using ..StructKnotPoint
using ..BaseNamedTrajectory


# -------------------------------------------------------------- #
# Get comps
# -------------------------------------------------------------- #

"""
    get_components(names, ::NamedTrajectory)

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
function get_component_names(traj::NamedTrajectory, comps::AbstractVector{Int})
    names = [n for n ∈ keys(filter_by_value(x -> issubset(x, comps), traj.components)) if n ∈ traj.names]
    if isempty(names)
        error("Component names not found in traj")
    else
        return names
    end
end

function get_component_name(traj::NamedTrajectory, comps::AbstractVector{Int})
    names = get_component_names(traj, comps)
    if length(names) == 1
        return names[1]
    else
        error("Multiple component names $(names) found.")
    end
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

# -------------------------------------------------------------- #
# Add/remove operations
# -------------------------------------------------------------- #

function extend_datavec(
    data::AbstractMatrix{R}, 
    ext_data::AbstractMatrix{R}
) where R <: Real
    @assert size(data, 2) == size(ext_data, 2)
    T = size(data, 2)
    dim = size(data, 1)
    ext_dim = size(ext_data, 1)
    new_datavec = zeros((dim + ext_dim) * T)
    for t in 1:T
        # fill original data
        copyto!(new_datavec, (t - 1) * (dim + ext_dim) + 1, data[:, t], 1, dim)
        # fill new data
        copyto!(new_datavec, (t - 1) * (dim + ext_dim) + dim + 1, ext_data[:, t], 1, ext_dim)
    end
    return new_datavec
end

"""
    add_components(traj, comps)

Add components to the trajectory.

Keyword arguments:
    - `type::Symbol`: The type of the component, can be `:state`, `:control`, `:slack`, or `:global`. Default is `:state`.
"""
function add_components(
    traj::NamedTrajectory,
    comps_data::NamedTuple{<:Any, <:Tuple};
    type::Symbol=:state,
    kwargs...
)
    if type == :global
        @assert all([c isa AbstractVector for c in values(comps_data)])
        @assert all([k ∉ keys(traj.gcomponents) for k in keys(comps_data)])

        gdata = vcat(traj.gdata, vcat(values(comps_data)...))

        # update global components
        start_idx = traj.gdim + 1
        gcomps_pairs = []
        for (k, v) in pairs(comps_data)
            stop_idx = start_idx + length(v) - 1
            push!(gcomps_pairs, k => (start_idx:stop_idx))
            start_idx = stop_idx + 1
        end
        gcomps = merge(traj.gcomponents, gcomps_pairs)

        println(typeof(gdata))

        return NamedTrajectory(
            traj.datavec,
            traj; 
            gdata=gdata, 
            gcomponents=gcomps,
            kwargs...
        )
    elseif type ∈ [:state, :control, :slack]
        @assert all([data isa AbstractMatrix for data in values(comps_data)])
        @assert all([size(data, 2) == traj.T for (name, data) in pairs(comps_data)])
        @assert all([name ∉ keys(traj.components) for (name, data) in pairs(comps_data)])

        # update components
        start_idx = traj.dim + 1
        comps_pairs = []
        for (k, v) in pairs(comps_data)
            stop_idx = start_idx + size(v, 1) - 1
            push!(comps_pairs, k => (start_idx:stop_idx))
            start_idx = stop_idx + 1
        end
        comps = merge(traj.components, comps_pairs)

        # update control names TODO: slack names are states?
        if type == :control
            controls = (traj.control_names..., keys(comps_data)...)
        else
            controls = traj.control_names
        end

        # update data
        datavec = extend_datavec(traj.data, vcat(values(comps_data)...))

        return NamedTrajectory(
            datavec,
            traj;
            components=comps,
            controls=controls,
            kwargs...
        )
    else
        throw(ArgumentError("Invalid type: $type. Must be one of :state, :control, :slack, or :global."))
    end
end

"""
    add_component(traj, name::Symbol, data::AbstractVecOrMat)

Add a component to the trajectory.

Keyword arguments:
    - `type::Symbol`: The type of the component, can be `:state`, `:control`, `:slack`, or `:global`. Default is `:state`.
"""
function add_component(
    traj::NamedTrajectory,
    name::Symbol,
    data::AbstractVecOrMat{Float64};
    type::Symbol=:state,
    kwargs...
)
    if type != :global && data isa AbstractVector
        @assert length(data) == traj.T "Data length must match trajectory T"
        comp_data = (; name => reshape(data, 1, traj.T),) 
    else
        comp_data = (; name => data,)
    end
    return add_components(traj, comp_data; type=type, kwargs...)
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
    new_timestep::Union{Nothing, Symbol}=nothing,
    new_controls::Union{Nothing, Symbol, Tuple{Vararg{Symbol}}}=nothing
)
    comps_data = NamedTuple(get_components(setdiff(traj.names, names), traj))
    gcomps_data = NamedTuple(get_components(setdiff(traj.gnames, names), traj))

    if traj.timestep in names
        @assert !isnothing(new_timestep) "New timestep must be provided if removing the timestep component"
        timestep = new_timestep
    else
        timestep = traj.timestep
    end

    controls = setdiff(traj.control_names, names)
    if !isnothing(new_timestep)
        controls = vcat(controls..., new_timestep)
    end
    if !isnothing(new_controls)
        if new_controls isa Symbol
            controls = vcat(controls..., new_controls)
        elseif new_controls isa Tuple
            controls = vcat(controls..., new_controls...)
        end
    end

    return NamedTrajectory(
        comps_data,
        gcomps_data;
        timestep=timestep,
        controls=Tuple(controls),
        bounds=NamedTuple(k => v for (k, v) in pairs(traj.bounds) if k ∉ names),
        initial=NamedTuple(k => v for (k, v) in pairs(traj.initial) if k ∉ names),
        final=NamedTuple(k => v for (k, v) in pairs(traj.final) if k ∉ names),
        goal=NamedTuple(k => v for (k, v) in pairs(traj.goal) if k ∉ names),
    )
end

# -------------------------------------------------------------- #
# Update operations
# -------------------------------------------------------------- #

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

Keyword arguments:
    - `type::Symbol`: The type of the datavec, can be `:data`, `:global`, or `:both`. Default is `global`.
"""
function update!(traj::NamedTrajectory, datavec::AbstractVector{Float64}; type=:data)
    if type == :data
        traj.datavec[:] = datavec
    elseif type == :global
        traj.gdata[:] = datavec
    elseif type == :both
        traj.datavec[:] = datavec[1:(traj.dim * traj.T)]
        traj.gdata[:] = datavec[(traj.dim * traj.T + 1):(traj.dim * traj.T + traj.gdim)]
    end
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
        timestep=add_suffix(traj.timestep, suffix),
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

# =========================================================================== #

@testitem "add component" begin
    using Random
    T = 10
    dim = 5
    data = randn(dim, T)
    traj = NamedTrajectory(data, (x = 1:3, y=4:4, z=5:5), timestep=:z)

    traj1 = add_component(traj, :b, randn(T))
    @test traj1.data[1:dim, :] == data

    traj2 = add_component(traj, :b, randn(2, T))
    @test traj2.data[1:dim, :] == data

    da = randn(2, T)
    db = randn(3, T)
    traj3 = add_components(traj, (a=da, b=db))
    @test traj3.data[1:dim, :] == data
    @test traj3[:a] == da
    @test traj3[:b] == db

    # test adding global component
    dg = randn(4)
    traj4 = add_component(traj, :g, dg; type=:global)
    @test traj4.data == data
    @test traj4.gdata == dg
end

@testitem "remove component" begin
    data = randn(5, 10)
    traj = NamedTrajectory(data, (x = 1:3, y=4:4, z=5:5), timestep=:z)
    traj1 = remove_component(traj, :y)
    for name in [:x, :z]
        @test name ∈ traj1.names
        @test traj1[name] == traj[name]
    end

    # test removing timestep 
    traj2 = remove_component(traj1, :z, new_timestep=:y)
    for name in [:x, :y]
        @test name ∈ traj2.names
        @test traj2[name] == traj1[name]
    end
    @test traj2.timestep == :y

    # remove multiple
    da = randn(2, T)
    db = randn(3, T)
    traj3 = add_components(traj, (a=da, b=db))
    @test remove_components(traj3, [:a, :b]) == traj

    # test removing global component

    traj4 = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z, 
        gdata=[1.0, 2.0], gcomponents=(g1=1:1, g2=2:2)
    )
    traj5 = remove_component(traj4, :g1)
    @test :g1 ∉ traj4.gnames
    @test traj5.gdata == [2.0]
    @test traj5.gcomponents == (g2=1:1)    
end

@testitem "update! data" begin
    using Random
    T = 10
    data = randn(5, T)
    gdata = [1.0, 2.0]
    orig_data = copy(data)
    orig_gdata = copy(gdata)
    traj = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z, 
        gdata=gdata, gcomponents=(g1=1:1, g2=2:2)
    )
    
    # update data
    update!(traj, :x, zeros(traj.dims[:x], T))
    @test traj[:x] == zeros(traj.dims[:x], T)
    @test traj[:y] == orig_data[traj.components[:y], :]
    @test traj[:z] == orig_data[traj.components[:z], :]

    # update datavec
    update!(traj, ones(traj.dim * traj.T))
    @test traj.datavec == ones(traj.dim * traj.T)
    @test traj.gdata == orig_gdata

    # update global data
    update!(traj, zeros(traj.gdim), type=:global)
    @test traj.datavec == ones(traj.dim * traj.T) # stays the same from before
    @test traj.gdata == zeros(traj.gdim) # changes

    # update both
    new_data = vcat(vec(orig_data), orig_gdata)
    update!(traj, new_data, type=:both)
    @test traj.data == orig_data
    @test traj.gdata = orig_gdata
end

# *** old ***

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