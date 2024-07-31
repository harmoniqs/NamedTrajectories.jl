module MethodsNamedTrajectory

export vec
export get_components
export add_component!
export remove_component
export remove_components
export update!
export update_bound!
export get_times
export get_timesteps
export get_duration

using OrderedCollections

using ..StructNamedTrajectory
using ..StructKnotPoint


function StructKnotPoint.KnotPoint(
    Z::NamedTrajectory,
    t::Int
)
    @assert 1 ≤ t ≤ Z.T
    timestep = get_timesteps(Z)[t]
    return KnotPoint(t, Z.data[:, t], timestep, Z.components, Z.names, Z.control_names)
end

"""
    vec(::NamedTrajectory)

Returns all variables of the trajectory as a vector, Z⃗.
"""
function Base.vec(Z::NamedTrajectory)
    return vcat(Z.datavec, values(Z.global_data)...)
end


"""
    length(::NamedTrajectory)

Returns the length of all variables of the trajectory.
"""
function Base.length(Z::NamedTrajectory)
    return Z.dim * Z.T + Z.global_dim
end


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


"""
    add_component!(traj, name::Symbol, data::AbstractVecOrMat; type={:state, :control})

Add a component to the trajectory.

NOTE: This function resizes the trajectory, so global components and components must be adjusted.
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
    else
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
        traj.control_names = (traj.control_names..., name)
        dim_dict[:controls] += dim
    else
        if :slacks ∉ keys(dim_dict)
            dim_dict[:slacks] = dim
        else
            dim_dict[:slacks] += dim
        end
    end

    traj.dims = NamedTuple(dim_dict)


    # update names

    traj.names = (traj.names..., name)


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
function remove_component(
    traj::NamedTrajectory, 
    name::Symbol; 
    new_control_name::Union{Nothing, Symbol}=nothing,
    new_control_names::Union{Nothing, Tuple{Vararg{Symbol}}}=nothing
)
    return remove_components(
        traj,
        [name];
        new_control_name=new_control_name,
        new_control_names=new_control_names
    )
end

"""
    remove_components(traj, names::Vector{Symbol})

Remove a set of components from the trajectory.
"""
function remove_components(
    traj::NamedTrajectory,
    names::AbstractVector{<:Symbol};
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
    return NamedTrajectory(comps, traj; new_control_names=new_control_names)
end

"""
    update!(traj, name::Symbol, data::AbstractMatrix{Float64})

Update a component of the trajectory.
"""
function update!(traj::NamedTrajectory, name::Symbol, data::AbstractMatrix{Float64})
    @assert name ∈ traj.names
    @assert size(data, 1) == traj.dims[name]
    @assert size(data, 2) == traj.T
    # TODO: test to see if updating both matrix and vec is necessary
    traj.data[traj.components[name], :] = data
    traj.datavec = vec(view(traj.data, :, :))
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
    new_bounds = OrderedDict(pairs(traj.bounds))
    new_bounds[name] = new_bound
    new_bounds = NamedTuple(new_bounds)
    traj.bounds = new_bounds
    return nothing
end



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

"""
    size(traj::NamedTrajectory) = (dim = traj.dim, T = traj.T)
"""
Base.size(traj::NamedTrajectory) = (dim = traj.dim, T = traj.T)

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

Returns the component of the trajectory with name `symb` or the property of the trajectory with name `symb`.
"""
function Base.getproperty(traj::NamedTrajectory, symb::Symbol)
    if symb ∈ fieldnames(NamedTrajectory)
        return getfield(traj, symb)
    else
        indices = traj.components[symb]
        return traj.data[indices, :]
    end
end

"""
    setproperty!(traj, name::Symbol, val::Any)

Dispatches setting properties of trajectories as either setting a component or a property via `setfield!` or `update!`.
"""
function Base.setproperty!(traj::NamedTrajectory, symb::Symbol, val::Any)
    if symb ∈ fieldnames(NamedTrajectory)
        setfield!(traj, symb, val)
    else
        update!(traj, symb, val)
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


end
