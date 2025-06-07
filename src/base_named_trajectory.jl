module BasedNamedTrajectory

using ..StructNamedTrajectory
using ..StructKnotPoint


Base.show(io::IO, Z::NamedTrajectory) = print(io, Z.components, ", T = ", Z.T)

"""
    length(Z::NamedTrajectory) = Z.dim * Z.T + Z.global_dim
"""
function Base.length(Z::NamedTrajectory)
    return Z.dim * Z.T + Z.global_dim
end

"""
    size(Z::NamedTrajectory) = (dim = Z.dim, T = Z.T, gdim = Z.gdim)
"""
Base.size(Z::NamedTrajectory) = (dim = Z.dim, T = Z.T, gdim = Z.gdim)

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
    elseif symb in traj.names
        indices = traj.components[symb]
        return view(traj.data, indices, :)
    elseif symb in traj.gnames
        indices = traj.gcomps[symb]
        return view(traj.gdata, indices, :)
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
# Get components, Equality
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

"""
    get_global_components(names, ::NamedTrajectory)

Returns a NamedTuple containing the names and corresponding data vector of the globals.
"""
function get_global_components(cnames::Union{Tuple, AbstractVector}, traj::NamedTrajectory)
    symbs = Tuple(c for c in cnames)
    vals = [traj.gdata[traj.gcomps[c]] for c ∈ cnames]
    return NamedTuple{symbs}(vals)
end

get_components(traj::NamedTrajectory) = get_components(traj.names, traj)

"""
    isequal(traj1::NamedTrajectory, traj2::NamedTrajectory)

Check if trajectories are equal w.r.t. data using `Base.isequal`
"""
function Base.isequal(traj1::NamedTrajectory, traj2::NamedTrajectory)
    # check components
    if !issetequal(traj1.names, traj2.names)
        return false
    end

    comps1 = get_components(traj1)
    comps2 = get_components(traj2)
    for (name, data1) in pairs(comps1)
        if !isequal(data1, comps2[name])
            return false
        end
    end

    # check global components
    if !issetequal(traj1.gnames, traj2.gnames)
        return false
    end

       
    return true
end

"""
    :(==)(traj1::NamedTrajectory, traj2::NamedTrajectory)

Check if trajectories are equal w.r.t. using `Base.:(==)`
"""
function Base.:(==)(traj1::NamedTrajectory, traj2::NamedTrajectory)
    return isequal(traj1, traj2)
end

# -------------------------------------------------------------- #
# Base math
# -------------------------------------------------------------- #

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