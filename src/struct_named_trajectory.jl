module StructNamedTrajectory

export NamedTrajectory
export BoundType

using OrderedCollections
using TestItems

const BoundType = Tuple{AbstractVector{<:Real}, AbstractVector{<:Real}}

function inspect_names(
    names::Tuple{Vararg{Symbol}},
    controls::Tuple{Vararg{Symbol}},
    initial::Tuple{Vararg{Symbol}},
    final::Tuple{Vararg{Symbol}},
    goal::Tuple{Vararg{Symbol}},
    bounds::Tuple{Vararg{Symbol}},
)
    for k ∈ controls
        @assert k ∈ names "Control $k not in component_data"
    end
    for k ∈ initial
        @assert k ∈ names "Initial $k not in component_data"
    end
    for k ∈ final
        @assert k ∈ names "Final $k not in component_data"
    end
    for k ∈ goal
        @assert k ∈ names "Goal $k not in component_data"
    end
    for k ∈ bounds
        @assert k ∈ names "Bound $k not in component_data"
    end
end

function inspect_dims_pairs(
    dims_pairs::Vector{Pair{Symbol, Int}},
    bounds::NamedTuple{bnames, <:Tuple{Vararg{BoundType}}} where bnames,
    initial::NamedTuple{inames, <:Tuple{Vararg{AbstractVector{R}}}} where inames,
    final::NamedTuple{fnames, <:Tuple{Vararg{AbstractVector{R}}}} where fnames,
    goal::NamedTuple{gnames, <:Tuple{Vararg{AbstractVector{R}}}} where gnames
) where R <: Real
    dims_tuple = NamedTuple(dims_pairs)
    for k in keys(bounds)
        @assert length(bounds[k][1]) == dims_tuple[k] "Bad bound for $k: ||$(bounds[k])|| ≠ $(dims_tuple[k])"
    end
    for k in keys(initial)
        @assert length(initial[k]) == dims_tuple[k] "Bad initial for $k: ||$(initial[k])|| ≠ $(dims_tuple[k])"
    end
    for k in keys(final)
        @assert length(final[k]) == dims_tuple[k] "Bad final for $k: ||$(final[k])|| ≠ $(dims_tuple[k])"
    end
    for k in keys(goal)
        @assert length(goal[k]) == dims_tuple[k] "Bad goal for ||$k: $(goal[k])|| ≠ $(dims_tuple[k])"
    end
end

"""
    NamedTrajectory constructor
"""
mutable struct NamedTrajectory{R <: Real}
    data::AbstractMatrix{R}
    datavec::AbstractVector{R}
    T::Int
    timestep::Union{Symbol,R}
    dim::Int
    dims::NamedTuple{dnames, <:Tuple{Vararg{Int}}} where dnames
    bounds::NamedTuple{bnames, <:Tuple{Vararg{BoundType}}} where bnames
    initial::NamedTuple{inames, <:Tuple{Vararg{AbstractVector{R}}}} where inames
    final::NamedTuple{fnames, <:Tuple{Vararg{AbstractVector{R}}}} where fnames
    goal::NamedTuple{gnames, <:Tuple{Vararg{AbstractVector{R}}}} where gnames
    components::NamedTuple{cnames, <:Tuple{Vararg{AbstractVector{Int}}}} where cnames
    global_data::NamedTuple{pnames, <:Tuple{Vararg{AbstractVector{R}}}} where pnames
    global_dim::Int
    global_dims::NamedTuple{gdnames, <:Tuple{Vararg{Int}}} where gdnames
    global_components::NamedTuple{gcnames, <:Tuple{Vararg{AbstractVector{Int}}}} where gcnames
    names::Tuple{Vararg{Symbol}}
    state_names::Tuple{Vararg{Symbol}}
    control_names::Tuple{Vararg{Symbol}}
end

"""
    NamedTrajectory(component_data; controls=(), timestep=nothing, bounds, initial, final, goal)

    # Arguments
    - `component_data::NamedTuple{names, <:Tuple{Vararg{AbstractMatrix{R}}}} where {names}`: Components data.
    - `controls`: The control variable in component_data, should be type of `Symbol` among `component_data`.
    - `timestep`: Discretizing time step in `component_data`, should be type of `Symbol` among `component_data`.
    - `bounds`: Bounds of the trajectory.
    - `initial`: Initial values.
    - `final`: Final values.
    - `goal`: Goal for the states.
"""
function NamedTrajectory(
    component_data::NamedTuple{names, <:Tuple{Vararg{AbstractMatrix{R}}}} where {names};
    controls::Union{Symbol, Tuple{Vararg{Symbol}}}=(),
    timestep::Union{Nothing,Symbol,R}=nothing,
    bounds=(;),
    initial=(;),
    final=(;),
    goal=(;),
    global_data=(;),
) where R <: Real
    controls = controls isa Symbol ? (controls,) : controls

    @assert !isempty(controls)
    @assert !isnothing(timestep)
    @assert timestep isa Symbol && timestep ∈ keys(component_data) ||
        timestep isa Real "timestep $(timestep)::$(typeof(timestep)) must be a symbol or real"
    
    names = Tuple(keys(component_data))
    inspect_names(names, controls, keys(initial), keys(final), keys(goal), keys(bounds))

    @assert all([
        bound isa Real ||
        bound isa AbstractVector{<:Real} ||
        bound isa Tuple{<:Real,<:Real} ||
        bound isa BoundType
            for bound ∈ bounds
    ])

    if timestep isa Symbol && !in(timestep, controls)
        controls = (controls..., timestep)
    end


    state_names = Tuple(k for k ∈ names if k ∉ controls)

    bounds_dict = OrderedDict{Symbol,Any}(pairs(bounds))

    for (name, bound) ∈ bounds_dict
        if bound isa Real
            bounds_dict[name] = (
                -fill(bound, size(component_data[name], 1)),
                fill(bound, size(component_data[name], 1))
            )
        elseif bound isa AbstractVector
            bounds_dict[name] = (-bound, bound)
        elseif bound isa Tuple{<:Real, <:Real}
            bounds_dict[name] = ([bound[1]], [bound[2]])
        end
    end
    bounds = NamedTuple(bounds_dict)

    component_data_pairs = []
    for (key, val) ∈ pairs(component_data)
        if val isa AbstractVector{<:Real}
            data = reshape(val, 1, :)
            push!(component_data_pairs, key => data)
        else
            push!(component_data_pairs, key => val)
        end
    end

    data = vcat([val for (key, val) ∈ component_data_pairs]...)
    dim, T = size(data)

    # store data matrix as view of datavec
    datavec = vec(data)
    data = reshape(view(datavec, :), :, T)

    dims_pairs = [(k => size(v, 1)) for (k, v) ∈ component_data_pairs]
    inspect_dims_pairs(dims_pairs, bounds, initial, final, goal)

    comp_pairs::Vector{Pair{Symbol, AbstractVector{Int}}} =
        [(dims_pairs[1][1] => 1:dims_pairs[1][2])]

    for (k, dim) in dims_pairs[2:end]
        k_range = comp_pairs[end][2][end] .+ (1:dim)
        push!(comp_pairs, k => k_range)
    end

    # add states and controls to dims

    dim_states = sum([dim for (k, dim) in dims_pairs if k ∉ controls])
    dim_controls = sum([dim for (k, dim) in dims_pairs if k ∈ controls])

    push!(dims_pairs, :states => dim_states)
    push!(dims_pairs, :controls => dim_controls)

    dims = NamedTuple(dims_pairs)

    # add states and controls to components

    temp_comp_tuple = NamedTuple(comp_pairs)
    states_comps = vcat([temp_comp_tuple[k] for k ∈ keys(component_data) if k ∉ controls]...)
    controls_comps = vcat([temp_comp_tuple[k] for k ∈ keys(component_data) if k ∈ controls]...)

    push!(comp_pairs, :states => states_comps)
    push!(comp_pairs, :controls => controls_comps)

    comps = NamedTuple(comp_pairs)

    # global dims

    global_dims_pairs = [(k => length(v)) for (k, v) ∈ pairs(global_data)]
    global_comps_pairs::Vector{Pair{Symbol, AbstractVector{Int}}} = []
    
    running_global_dim = 0
    for (k, v) ∈ global_dims_pairs
        # offset by datavec
        k_value = (dim * T + running_global_dim) .+ (1:v)
        push!(global_comps_pairs, k => k_value)
        running_global_dim += v
    end

    global_comps = NamedTuple(global_comps_pairs)
    global_dims = NamedTuple(global_dims_pairs)
    global_dim = sum(values(global_dims), init=0)

    return NamedTrajectory{R}(
        data,
        datavec,
        T,
        timestep,
        dim,
        dims,
        bounds,
        initial,
        final,
        goal,
        comps,
        global_data,
        global_dim,
        global_dims,
        global_comps,
        names,
        state_names,
        controls
    )
end

"""
    NamedTrajectory(component_data; kwargs...)

    # Arguments
    - `component_data::NamedTuple`: Components data. Values should be of type `Union{AbstractVector{<:Real}, AbstractMatrix{<:Real}}`.
    - `kwargs...`: The other key word arguments.
"""
function NamedTrajectory(
    component_data::NamedTuple;
    kwargs...
)
    @assert all([v isa AbstractMatrix || v isa AbstractVector for v ∈ values(component_data)])
    @assert all([eltype(v) <: Real for v ∈ values(component_data)]) "eltypes are $([eltype(v) for v ∈ values(component_data)])"
    vals = [v isa AbstractVector ? reshape(v, 1, :) : v for v ∈ values(component_data)]
    component_data = NamedTuple([(k => v) for (k, v) ∈ zip(keys(component_data), vals)])
    return NamedTrajectory(component_data; kwargs...)
end




"""
    NamedTrajectory(datavec, T, components)

"""
function NamedTrajectory(
    datavec::AbstractVector{R},
    T::Int,
    components::NamedTuple{
        names,
        <:Tuple{Vararg{AbstractVector{Int}}}
    } where names;
    timestep::Union{Nothing,Symbol,R}=nothing,
    controls::Union{Symbol, Tuple{Vararg{Symbol}}}=(),
    bounds=(;),
    initial=(;),
    final=(;),
    goal=(;),
    global_data=(;),
) where R <: Real
    controls = (controls isa Symbol) ? (controls,) : controls

    @assert !isempty(controls) "must specify at least one control"
    @assert !isnothing(timestep) "must specify a time step size"
    @assert timestep isa Symbol && timestep ∈ keys(components) ||
        timestep isa Real

    names = Tuple(keys(components))
    inspect_names(names, controls, keys(initial), keys(final), keys(goal), keys(bounds))

    @assert all([
        (bound isa Real) ||
        (bound isa AbstractVector{<:Real}) ||
        (bound isa Tuple{<:Real,<:Real}) ||
        (bound isa BoundType)
        for bound ∈ bounds
    ])
    if timestep isa Symbol && !in(timestep, controls)
        controls = (controls..., timestep)
    end

    bounds_dict = OrderedDict(pairs(bounds))
    for (name, bound) ∈ bounds_dict
        if bound isa AbstractVector
            bounds_dict[name] = (-bound, bound)
        end
    end
    bounds = NamedTuple(bounds_dict)

    data = reshape(view(datavec, :), :, T)
    dim = size(data, 1)

    @assert all([isa(components[k], AbstractVector{Int}) for k in keys(components)])
    @assert vcat([components[k] for k in keys(components)]...) == 1:dim

    dim_pairs = [(k => length(components[k])) for k in keys(components)]
    inspect_dims_pairs(dims_pairs, bounds, initial, final, goal)

    dim_states = sum([dim for (k, dim) ∈ dim_pairs if k ∉ controls])
    dim_controls = sum([dim for (k, dim) ∈ dim_pairs if k ∈ controls])

    push!(dim_pairs, :states => dim_states)
    push!(dim_pairs, :controls => dim_controls)

    dims = NamedTuple(dim_pairs)

    state_names = Tuple(k for k ∈ names if k ∉ controls)

    # global dims
    
    global_dims_pairs = [(k => length(v)) for (k, v) ∈ pairs(global_data)]
    global_comps_pairs::Vector{Pair{Symbol, AbstractVector{Int}}} = []
    
    running_global_dim = 0
    for (k, v) ∈ global_dims_pairs
        # offset by datavec
        push!(global_comps_pairs, k => (dim * T + running_global_dim) .+ 1:v)
        running_global_dim += v
    end

    global_comps = NamedTuple(global_comps_pairs)
    global_dims = NamedTuple(global_dims_pairs)
    global_dim = sum(values(global_dims), init=0)

    return NamedTrajectory{R}(
        data,
        datavec,
        T,
        timestep,
        dim,
        dims,
        bounds,
        initial,
        final,
        goal,
        components,
        global_data,
        global_dim,
        global_dims,
        global_comps,
        names,
        state_names,
        controls
    )
end

"""
    NamedTrajectory(component_data; controls=(), timestep=nothing, bounds, initial, final, goal)

    # Arguments
    - `datavec::AbstractVector{R} where R <: Real`: Trajectory data.
    - `traj`: Constructed `NamedTrajectory`.
"""
function NamedTrajectory(
    datavec::AbstractVector{R},
    traj::NamedTrajectory
) where R <: Real
    return NamedTrajectory(
        datavec,
        traj.global_data,
        traj
    )
end

function NamedTrajectory(
    datavec::AbstractVector{R},
    global_data::NamedTuple{pnames, <:Tuple{Vararg{AbstractVector{R}}}} where pnames,
    traj::NamedTrajectory
) where R <: Real
    @assert length(datavec) == length(traj.datavec)
    for (k, v) ∈ pairs(global_data)
        @assert length(v) == traj.global_dims[k] "$k: $(length(v)) != $(traj.global_dims[k])"
    end

    # collecting here to prevent overlapping views
    # TODO: is this necessary?
    datavec = collect(datavec)

    data = reshape(view(datavec, :), :, traj.T)

    return NamedTrajectory{R}(
        data,
        datavec,
        traj.T,
        traj.timestep,
        traj.dim,
        traj.dims,
        traj.bounds,
        traj.initial,
        traj.final,
        traj.goal,
        traj.components,
        global_data,
        traj.global_dim,
        traj.global_dims,
        traj.global_components,
        traj.names,
        traj.state_names,
        traj.control_names
    )
end

"""
    NamedTrajectory(data, traj)

    # Arguments
    - `data`: Trajectory data.
    - `traj`: Constructed `NamedTrajectory`.
"""
function NamedTrajectory(
    data::AbstractMatrix{R},
    traj::NamedTrajectory
) where R <: Real
    @assert size(data) == size(traj.data)

    # collecting here to prevent overlapping views
    # TODO: is this necessary?
    datavec = vec(collect(data))

    data = reshape(view(datavec, :), :, traj.T)

    return NamedTrajectory{R}(
        data,
        datavec,
        traj.T,
        traj.timestep,
        traj.dim,
        traj.dims,
        traj.bounds,
        traj.initial,
        traj.final,
        traj.goal,
        traj.components,
        traj.global_data,
        traj.global_dim,
        traj.global_dims,
        traj.global_components,
        traj.names,
        traj.state_names,
        traj.control_names
    )
end

"""
    NamedTrajectory(data, components; kwargs...)

    # Arguments
    - `data::AbstractMatrix{R}`: Trajectory data.
    - `components::NamedTuple{names, <:Tuple{Vararg{AbstractVector{Int}}}} where names`: components data.
    - `kwargs...` : The other key word arguments.
"""
function NamedTrajectory(
    data::AbstractMatrix{R},
    components::NamedTuple{
        names,
        <:Tuple{Vararg{AbstractVector{Int}}}
    } where names;
    kwargs...
) where R <: Real
    T = size(data, 2)
    datavec = vec(data)
    return NamedTrajectory(datavec, T, components; kwargs...)
end

"""
    NamedTrajectory(component_data; controls=(), timestep=nothing, bounds, initial, final, goal)

    # Arguments
    - `comps::NamedTuple{names, <:Tuple{Vararg{AbstractMatrix{R}}}} where {names}`: components data.
    - `traj`: Constructed NamedTrajectory.
    - `goal`: Goal for the states.
"""
function NamedTrajectory(
    comps::NamedTuple{
        names,
        <:Tuple{Vararg{AbstractMatrix{R}}}
    } where names,
    traj::NamedTrajectory;
    new_timestep::Union{Nothing, R}=nothing,
    new_control_names::Tuple{Vararg{Symbol}}=()
) where R <: Real
    @assert all([k ∈ traj.names for k ∈ keys(comps)])

    control_names = (
        [name for name ∈ traj.control_names if name ∈ keys(comps) && name ∉ new_control_names]...,
        new_control_names...
    )
    @assert !isempty(control_names) "must specify at least one control"

    if traj.timestep isa Symbol && isnothing(new_timestep)
        @assert traj.timestep ∈ keys(comps) "timestep symbol must be in components"
    end

    bounds = NamedTuple([(k => traj.bounds[k]) for k ∈ keys(comps) if k ∈ keys(traj.bounds)])
    initial = NamedTuple([(k => traj.initial[k]) for k ∈ keys(comps) if k ∈ keys(traj.initial)])
    final = NamedTuple([(k => traj.final[k]) for k ∈ keys(comps) if k ∈ keys(traj.final)])
    goal = NamedTuple([(k => traj.goal[k]) for k ∈ keys(comps) if k ∈ keys(traj.goal)])
    # global comps are separate from comps
    global_data = deepcopy(traj.global_data)

    return NamedTrajectory(
        comps;
        controls=control_names,
        timestep=isnothing(new_timestep) ? traj.timestep : new_timestep,
        bounds=bounds,
        initial=initial,
        final=final,
        goal=goal,
        global_data=global_data,
    )

end

# =========================================================================== #

@testitem "testing constructor" begin
    # define number of timesteps and timestep
    T = 10
    dt = 0.1

    components = (
    x = rand(3, T),
    u = rand(2, T),
    Δt = fill(dt, 1, T),
    )

    timestep = 0.1
    control = :u

    # some global params as a NamedTuple
    params = (
        α = rand(1),
        β = rand(1)
    )

    traj = NamedTrajectory(
        components; 
        timestep=timestep, 
        controls=control, 
        global_data=params
    )

    @test traj.global_data == params
end


end
