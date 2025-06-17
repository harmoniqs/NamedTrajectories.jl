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
export update_bound!

export merge

export add_suffix
export remove_suffix
export get_suffix

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
        @assert all([k ∉ keys(traj.global_components) for k in keys(comps_data)])

        global_data = vcat(traj.global_data, vcat(values(comps_data)...))

        # update global components
        start_idx = traj.global_dim + 1
        gcomps_pairs = []
        for (k, v) in pairs(comps_data)
            stop_idx = start_idx + length(v) - 1
            push!(gcomps_pairs, k => (start_idx:stop_idx))
            start_idx = stop_idx + 1
        end
        gcomps = merge(traj.global_components, gcomps_pairs)

        return NamedTrajectory(
            traj; 
            datavec=traj.datavec,
            global_data=global_data, 
            global_components=gcomps,
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
            traj;
            datavec=datavec,
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
    gcomps_data = NamedTuple(get_components(setdiff(traj.global_names, names), traj))

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
        traj.global_data[:] = datavec
    elseif type == :both
        traj.datavec[:] = datavec[1:(traj.dim * traj.T)]
        traj.global_data[:] = datavec[(traj.dim * traj.T + 1):(traj.dim * traj.T + traj.global_dim)]
    end
    return nothing
end

""" 
    update_bound!(traj, name, new_bound)

Update the bound of a component of the trajectory.
"""
function update_bound!(
    traj::NamedTrajectory,
    name::Symbol,
    new_bound
)
    @assert name in keys(traj.bounds) "update_bound! requires existing bound"
    # reuse processing
    new_bounds = StructNamedTrajectory.get_bounds_from_dims((; name => new_bound,), traj.dims)
    traj.bounds = merge(traj.bounds, new_bounds)
    return nothing
end


# -------------------------------------------------------------- #
# Merge operations
# -------------------------------------------------------------- #

"""
    merge(traj1::NamedTrajectory, traj2::NamedTrajectory)
    merge(trajs::AbstractVector{<:NamedTrajectory})

Returns a new NamedTrajectory object by merging `NamedTrajectory` objects. 

Merge names are used to specify which components to merge by index. If no merge names are provided, all components are merged and name collisions are not allowed. If merge names are provided, the names are merged using the data from the index provided in the merge names.

# Keyword Arguments
- `timestep::Symbol`: The timestep symbol to use for free time problems. Default to the last trajectory.
- `merge_names::Union{Nothing, NamedTuple{<:Any, <:Tuple{Vararg{Int}}}}=nothing`: The names to merge by index.
"""
function Base.merge(traj1::NamedTrajectory, traj2::NamedTrajectory; kwargs...)
    return merge([traj1, traj2]; kwargs...)
end

function Base.merge(
    trajs::AbstractVector{<:NamedTrajectory};
    merge_names::NamedTuple{<:Any, <:Tuple{Vararg{Int}}}=NamedTuple(),
    timestep::Symbol=trajs[end].timestep,
    timesteps::AbstractVector{<:Real}=get_timesteps(trajs[end])
)
    if length(trajs) < 2
        throw(ArgumentError("At least two trajectories must be provided"))
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
 
    # add timesteps
    if timestep ∉ keys(components)
        components = merge_outer(components, (; timestep => timesteps,))
    end

    # merge global data
    global_names = [[s for s ∈ traj.global_names if s ∉ names] for (traj, names) in zip(trajs, drop_names)]
    global_components = merge_outer([get_components(g, t) for (g, t) ∈ zip(global_names, trajs)])

    return NamedTrajectory(
        components,
        global_components,
        timestep=timestep,
        controls=merge_outer([Tuple(c) for c in control_names]),
        bounds=merge_outer(
            [drop(traj.bounds, names) for (traj, names) in zip(trajs, drop_names)]),
        initial=merge_outer(
            [drop(traj.initial, names) for (traj, names) in zip(trajs, drop_names)]),
        final=merge_outer(
            [drop(traj.final, names) for (traj, names) in zip(trajs, drop_names)]),
        goal=merge_outer(
            [drop(traj.goal, names) for (traj, names) in zip(trajs, drop_names)]),
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
    return NamedTrajectory(
        add_suffix(traj.names, traj, suffix),
        add_suffix(traj.global_names, traj, suffix);
        controls=add_suffix(traj.control_names, suffix),
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
    timestep::Symbol=traj.timestep,
    remove::Bool=false
)
    # exclude from suffix removal a timestep not ending in suffix
    if endswith(timestep, suffix)
        exclude = Symbol[]
        new_timestep = remove ? remove_suffix(timestep, suffix) : timestep
    else
        exclude = [timestep]
        new_timestep = timestep
    end

    # always select timestep data
    component_names = [n for n in traj.names if endswith(n, suffix) || n == timestep]
    components = get_components(component_names, traj)
    if remove
        components = remove_suffix(components, suffix; exclude=exclude)
    end

    gcomponent_names = [n for n in traj.global_names if endswith(n, suffix)]
    global_components = get_components(gcomponent_names, traj)
    if remove
        global_components = remove_suffix(global_components, suffix; exclude=exclude)
    end

    if isempty(component_names)
        error("No components found with suffix '$suffix'")
    end 

    return NamedTrajectory(
        components,
        global_components,
        timestep=new_timestep,
        bounds=get_suffix(traj.bounds, suffix, remove=remove),
        initial=get_suffix(traj.initial, suffix, remove=remove),
        final=get_suffix(traj.final, suffix, remove=remove),
        goal=get_suffix(traj.goal, suffix, remove=remove)
    )
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
    @test traj4.global_data == dg
end

@testitem "remove component" begin
    T = 10
    data = randn(5, T)
    traj = NamedTrajectory(data, (x = 1:3, y=4:4, z=5:5), timestep=:z)
    traj1 = remove_component(traj, :y)
    for name in [:x, :z]
        @test name ∈ traj1.names
        @test traj1[name] == traj[name]
    end

    # test removing timestep 
    traj2 = remove_component(traj, :z, new_timestep=:y)
    for name in [:x, :y]
        @test name ∈ traj2.names
        @test traj2[name] == traj[name]
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
        global_data=[1.0, 2.0], global_components=(g1=1:1, g2=2:2)
    )
    traj5 = remove_component(traj4, :g1)
    @test :g1 ∉ traj5.global_names
    @test traj5.global_data == [2.0]
    @test traj5.global_components == (g2 = 1:1,)    
end

@testitem "update! data" begin
    using Random
    T = 10
    data = randn(5, T)
    global_data = [1.0, 2.0]
    orig_data = copy(data)
    orig_global_data = copy(global_data)
    traj = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z, 
        global_data=global_data, global_components=(g1=1:1, g2=2:2)
    )
    
    # update data
    update!(traj, :x, zeros(traj.dims[:x], T))
    @test traj[:x] == zeros(traj.dims[:x], T)
    @test traj[:y] == orig_data[traj.components[:y], :]
    @test traj[:z] == orig_data[traj.components[:z], :]

    # update datavec
    update!(traj, ones(traj.dim * traj.T))
    @test traj.datavec == ones(traj.dim * traj.T)
    @test traj.global_data == orig_global_data

    # update global data
    update!(traj, zeros(traj.global_dim), type=:global)
    @test traj.datavec == ones(traj.dim * traj.T) # stays the same from before
    @test traj.global_data == zeros(traj.global_dim) # changes

    # update both
    new_data = vcat(vec(orig_data), orig_global_data)
    update!(traj, new_data, type=:both)
    @test traj.data == orig_data
    @test traj.global_data == orig_global_data
end

@testitem "update trajectory components via view" begin
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

@testitem "update bound" begin
    using Random

    data = randn(5, 10)
    traj = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z, 
        bounds=(x = 1,)
    )
    update_bound!(traj, :x, 2)
    @test traj.bounds[:x] == ([-2.0, -2.0, -2.0], [2.0, 2.0, 2.0])

    update_bound!(traj, :x, (0, 3))
    @test traj.bounds[:x] == ([0.0, 0.0, 0.0], [3.0, 3.0, 3.0])

    update_bound!(traj, :x, [1.0, 1.0, 1.0])
    @test traj.bounds[:x] == ([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0])
end

@testitem "merge" begin
    using Random
    T = 10
    traj1 = NamedTrajectory(randn(5, T), (x=1:3, y=4:4, z=5:5), timestep=:z)
    traj2 = NamedTrajectory(randn(5, T), (a=1:3, b=4:4, c=5:5), timestep=:c)
    
    traj3 = merge(traj1, traj2)
    @test traj3.timestep == traj2.timestep
    @test issetequal(traj3.names, vcat(traj1.names..., traj2.names...))

    traj4 = merge(traj1, traj2, timestep = :y)
    @test traj4.timestep == :y
    @test issetequal(traj3.names, vcat(traj1.names..., traj2.names...))

    # TODO: test merge_name collisions and indices
    # TODO: test merge of vector of trajectories
end

@testitem "returning times" begin
    data = randn(5, 10)
    traj = NamedTrajectory(data, (x=1:3, y=4:4, z=5:5), timestep=:z)
    @test get_times(traj) ≈ [0.0, cumsum(data[end, 1:end-1])...]
end

@testitem "suffix tests" begin
    T = 10
    data = randn(5, T)
    traj = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z, 
        global_data=[1.0, 2.0], global_components=(a = 1:2, ), bounds = (x = 1.0, y = 2.0,)
    )    
    suffix = "_test"
    traj_suffixed = add_suffix(traj, suffix)
    @test all(endswith.(keys(traj_suffixed.components), suffix))
    @test all(endswith.(keys(traj_suffixed.global_components), suffix))
    @test traj_suffixed.timestep == add_suffix(traj.timestep, suffix)
    @test traj_suffixed.global_data == traj.global_data
    @test traj_suffixed.global_components == add_suffix(traj.global_components, suffix)
    @test traj_suffixed.data == traj.data
    @test traj_suffixed.names == add_suffix(traj.names, suffix)
    @test traj_suffixed.global_names == add_suffix(traj.global_names, suffix)
    @test traj_suffixed.control_names == add_suffix(traj.control_names, suffix)
    @test traj_suffixed.bounds == add_suffix(traj.bounds, suffix)

    # test removing suffix
    traj2 = NamedTrajectory(randn(5, T), (x = 1:3, y=4:4, z=5:5), timestep=:z,)
    merge_traj = merge(traj_suffixed, traj2)
    # need to choose the right timestep to keep
    traj_unsuffixed = get_suffix(
        merge_traj, suffix, timestep=add_suffix(:z, suffix), remove=true
    )
    @test traj_unsuffixed == traj
    traj_got = get_suffix(merge_traj, suffix, remove=false)
    # keeps the original timestep, so remove it
    @test traj_got.timestep == traj.timestep
    @test remove_component(traj_got, :z, new_timestep=add_suffix(:z, suffix)) == traj_suffixed
end

end