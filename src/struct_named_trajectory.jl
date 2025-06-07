module StructNamedTrajectory

export NamedTrajectory
export BoundType

using OrderedCollections
using TestItems

const DimType = Tuple{Vararg{Int}}
const BoundType = Tuple{AbstractVector{<:Real}, AbstractVector{<:Real}}

# UnitRange{Int} enforces nonallocating @views with indexing
const ComponentType = Tuple{Vararg{UnitRange{Int}}}

# TODO: Types and check on R <: Real for initial, final, goal


# ---------------------------------------------------------------------------- #
# Named Trajectory
# ---------------------------------------------------------------------------- #

"""
    NamedTrajectory

Container for trajectory optimization problems, which includes the trajectory data, bounds,
dimensions, initial and final conditions, goal states, and components.

This struct is designed to hold trajectory data in a named format, allowing for easy access 
to knot points by `Symbol`.

NamedTrajectory is designed to make allocation-free access easy to write.
"""
mutable struct NamedTrajectory{
    R <: Real,
    DNames, DTypes <: DimType,
    BNames, BTypes <: Tuple{Vararg{BoundType}},
    INames, ITypes <: Tuple,
    FNames, FTypes <: Tuple,
    GNames, GTypes <: Tuple,
    CNames, CTypes <: ComponentType,
    N <: Tuple{Vararg{Symbol}},
    SN <: Tuple{Vararg{Symbol}},
    CN <: Tuple{Vararg{Symbol}},
    GDNames, GDTypes <: DimType,
    GCNames, GCTypes <: ComponentType,
    GN <: Tuple{Vararg{Symbol}},
}
    datavec::Vector{R}
    T::Int
    timestep::Symbol
    dim::Int
    dims::NamedTuple{DNames, DTypes}
    bounds::NamedTuple{BNames, BTypes}
    initial::NamedTuple{INames, ITypes}
    final::NamedTuple{FNames, FTypes}
    goal::NamedTuple{GNames, GTypes}
    components::NamedTuple{CNames, CTypes}
    names::N
    state_names::SN
    control_names::CN
    gdata::Vector{R}
    gdim::Int
    gdims::NamedTuple{GDNames, GDTypes}
    gcomponents::NamedTuple{GCNames, GCTypes}
    gnames::GN
end

"""
    NamedTrajectory(datavec, components, T)

Construct a named trajectory from a data vector, components, and knot points.
"""
function NamedTrajectory(
    datavec::AbstractVector{R},
    comps::NamedTuple{N, <:ComponentType} where N,
    T::Int;
    timestep::Symbol=:Δt,
    controls::Union{Symbol, Tuple{Vararg{Symbol}}}=timestep,
    bounds=NamedTuple(),
    initial=NamedTuple(),
    final=NamedTuple(),
    goal=NamedTuple(),
    gdata::AbstractVector{R}=R[],
    gcomps::NamedTuple{GN, <:ComponentType} where GN=NamedTuple(),
) where R <: Real
    @assert :data ∉ keys(comps) "data is a reserved name"
    @assert isdisjoint(keys(comps), keys(gcomps)) "components and global components should use unique names"

    @assert timestep isa Symbol && timestep ∈ keys(comps)

    controls = controls isa Symbol ? (controls,) : controls

    # timestep is a control
    if timestep isa Symbol && !in(timestep, controls)
        controls = (controls..., timestep)
    end
    
    names = Tuple(keys(comps))
    inspect_names(names, controls, keys(initial), keys(final), keys(goal), keys(bounds))
    states = Tuple(k for k ∈ names if k ∉ controls)

    # save data matrix as view of datavec
    data = reshape(view(datavec, :), :, T)
    dim = size(data, 1)

    # save dims
    dims_pairs = [(k => length(v)) for (k, v) ∈ pairs(comps)]
    dims = NamedTuple(dims_pairs)

    # process and save bounds
    bounds = get_bounds_from_dims(bounds, dims)

    # check data
    inspect_dims_pairs(dims_pairs, bounds, initial, final, goal)

    # global data 
    gdim = length(gdata)
    gdims = NamedTuple([(k => length(v)) for (k, v) ∈ pairs(gcomps)])
    gnames = Tuple(keys(gcomps))

    # check global data
    @assert gdim == sum(values(gdims), init=0) "invalid global data dims"

    return NamedTrajectory(
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
        names,
        states,
        controls,
        gdata,
        gdim,
        gdims,
        gcomps,
        gnames
    )
end

"""
    NamedTrajectory(component_data, gcomponents_data)

Construct a `NamedTrajectory` from component data and global component data
"""
function NamedTrajectory(
    comps_data::NamedTuple{N, <:Tuple{Vararg{AbstractMatrix{R}}}} where N,
    gcomps_data::NamedTuple{GN, <:Tuple{Vararg{AbstractVector{R}}}} where GN;
    kwargs...
) where R <: Real
    # unpack data
    data = vcat([val for (key, val) ∈ pairs(comps_data)]...)
    dim, T = size(data)

    # save components of data matrix
    dims_pairs = [(k => size(v, 1)) for (k, v) ∈ pairs(comps_data)]
    comps_pairs = [(dims_pairs[1][1] => 1:dims_pairs[1][2])]
    for (k, dim) in dims_pairs[2:end]
        k_range = comps_pairs[end][2][end] .+ (1:dim)
        push!(comps_pairs, k => k_range)
    end
    comps = NamedTuple(comps_pairs)

    # unpack global data
    gdata = vcat([val for (key, val) ∈ pairs(gcomps_data)]...)

    # save global componets
    if !isempty(gcomps_data)
        gdims_pairs = [(k => length(v)) for (k, v) ∈ pairs(gcomps_data)]
        gcomps_pairs = [(gdims_pairs[1][1] => 1:gdims_pairs[1][2])]
        for (k, v) ∈ gdims_pairs[2:end]
            # offset within gdata
            k_value = gcomps_pairs[end][2][end] .+ (1:v)
            push!(gcomps_pairs, k => k_value)
        end
        gcomps = NamedTuple(gcomps_pairs)

        return NamedTrajectory(
            vec(data), comps, T; 
            gdata=gdata, gcomps=gcomps, kwargs...
        )
    else
        return NamedTrajectory(vec(data), comps, T; kwargs...)
    end
end

"""
    NamedTrajectory(component_data)

Construct a `NamedTrajectory` from component data.
"""
function NamedTrajectory(
    comps_data::NamedTuple{N, <:Tuple{Vararg{AbstractMatrix{R}}}} where N;
    kwargs...
) where R <: Real
    return NamedTrajectory(
        comps_data,
        NamedTuple();
        kwargs...
    )
end

"""
    NamedTrajectory(component_data::NamedTuple, timestep; kwargs...)

Construct a `NamedTrajectory` from mixed Matrix/Vector component data.
"""
function NamedTrajectory(
    comps_data::NamedTuple;
    kwargs...
)
    @assert all([v isa AbstractMatrix || v isa AbstractVector for v ∈ values(comps_data)])
    vals = [v isa AbstractVector ? reshape(v, 1, :) : v for v ∈ values(comps_data)]
    comps_data = NamedTuple([(k => v) for (k, v) ∈ zip(keys(comps_data), vals)])
    return NamedTrajectory(comps_data, timestep; kwargs...)
end

"""
    NamedTrajectory(data, traj)

Construct a `NamedTrajectory` from a datavec and an existing `NamedTrajectory`.
"""
function NamedTrajectory(
    datavec::AbstractVector{R},
    traj::NamedTrajectory;    
    gdata::AbstractVector{R}=traj.gdata,
) where R <: Real
    @assert length(datavec) == length(traj.datavec)
    @assert length(gdata) == length(traj.gdata)

    return NamedTrajectory(
        datavec,
        traj.components,
        traj.T;
        timestep=traj.timestep,
        controls=traj.control_names,
        bounds=traj.bounds,
        initial=traj.initial,
        final=traj.final,
        goal=traj.goal,
        gdata=gdata,
        gdim=traj.gdim,
        gdims=traj.gdims,
        gcomponents=traj.gcomponents,
        gnames=traj.gnames
    )
end

"""
    NamedTrajectory(data, components; kwargs...)

Construct a `NamedTrajectory` from a data matrix and components.
"""
function NamedTrajectory(
    data::AbstractMatrix{R},
    comps::NamedTuple{N, <:ComponentType} where N;
    kwargs...
) where R <: Real
    T = size(data, 2)
    datavec = vec(data)
    return NamedTrajectory(datavec, comps, T; kwargs...)
end

# ----------------------------------------------------------------------------- #
# Named trajectory data processing
# ----------------------------------------------------------------------------- #

function get_bounds_from_dims(
    bounds::NamedTuple,
    dims::NamedTuple{N, <:Tuple{Vararg{Int}}} where N,
)   
    bounds_dict = OrderedDict{Symbol,Any}(pairs(bounds))
    for (name, bound) ∈ bounds_dict
        bdim = dims[name]
        if bound isa Real
            vbound = fill(bound, bdim)
            bounds_dict[name] = (-vbound, vbound)
        elseif bound isa Tuple{<:Real, <:Real}
            bounds_dict[name] = (fill(bound[1], bdim), fill(bound[2], bdim))
        elseif bound isa AbstractVector
            if length(bound) != bdim 
                throw(ArgumentError("Bound $name has wrong length: $(length(bound)) != $bdim"))
            end
            bounds_dict[name] = (-bound, bound)
        elseif bound isa BoundType
            bounds_dict[name] = bound
        else
            throw(ArgumentError("Invalid bound type for $name: $(typeof(bound))"))
        end
    end
    return NamedTuple(bounds_dict)
end

"""
    inspect_names(names, controls, initial, final, goal, bounds)

Check for missing names in the trajectory components.
"""
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

"""
    inspect_dims_pairs(dims_pairs, bounds, initial, final, goal)

Check for proper formatting of trajectory components.
"""
function inspect_dims_pairs(
    dims_pairs::Vector{Pair{Symbol, Int}},
    bounds::NamedTuple{bname, <:Tuple{Vararg{BoundType}}} where bname,
    initial::NamedTuple{iname, <:Tuple{Vararg{AbstractVector{R}}}} where iname,
    final::NamedTuple{fname, <:Tuple{Vararg{AbstractVector{R}}}} where fname,
    goal::NamedTuple{gname, <:Tuple{Vararg{AbstractVector{R}}}} where gname
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

# =========================================================================== #"

@testitem "Construct from data matrix and comps" begin
    n = 5
    T = 10
    data = randn(n, T)
    traj = NamedTrajectory(data, (x = 1:3, y=4:4, Δt=5:5))
    @test traj.data ≈ data
    @test traj.timestep == :Δt
    @test traj.dim == n
    @test traj.T == T
    @test traj.names == [:x, :y, :Δt]

    traj = NamedTrajectory(data, (x = 1:3, y=4:4, z=5:5), timestep=:z)
    @test traj.data ≈ data
    @test traj.timestep == :z
    @test traj.dim == n
    @test traj.T == T
    @test traj.names == [:x, :y, :z]
end

@testitem "Construct from component data" begin
    # define number of timesteps and timestep
    T = 10
    dt = 0.1

    dim = 6
    comps_data = (
        x = rand(3, T),
        u = rand(2, T),
        Δt = fill(dt, 1, T),
    )

    timestep = :Δt
    control = :u

    # some global params as a NamedTuple
    gdim = 2
    gcomps_data = (
        α = rand(1),
        β = rand(1)
    )

    # test global
    # ---
    traj = NamedTrajectory(
        comps_data,
        gcomps_data;
        timestep=timestep, 
        controls=control
    )

    @test traj.T == T
    @test traj.dim == dim
    @test length(traj.gdata) == gdim
    @test traj.names = [:x, :u, :Δt]
    @test traj.state_names == (:x,)
    @test traj.control_names == (:u, :Δt)
    @test traj.gnames == [:α, :β]

    comps_res = NamedTuple([(k => traj.data[v, :]) for (k, v) in pairs(traj.components)]) 
    @test comps_res == comps_data

    gres = NamedTuple([(k => traj.gdata[v]) for (k, v) in pairs(traj.gcomponents)])
    @test gres == gcomps_data

    # ignore global
    # ---
    traj = NamedTrajectory(comps_data)

    @test traj.T == T
    @test traj.dim == dim
    @test traj.names = [:x, :u, :Δt]
    @test traj.state_names == (:x,)
    @test traj.control_names == (:u, :Δt)
    @test isempty(traj.gnames)

    comps_res = NamedTuple([(k => traj.data[v, :]) for (k, v) in pairs(traj.components)]) 
    @test comps_res == comps_data

    @test isempty(traj.gdata)
end

@testitem "Test bounds" begin
    data = randn(10, 10)

    # Test: (real, real), (vec, vec), real, vec
    xlow = [-1, -2, -3]
    xupp = [1, 2, 3]
    xtuple = (xlow, xupp)
    ylow = 0
    yup = 1
    ytuple = (ylow, yup)
    zval = 1
    wval = [5, 6]
    traj = NamedTrajectory(
        data, 
        (Δt=1:1, x=2:4, y=5:5, z=6:8, w=9:10),
        bounds=(x=xtuple, y=ytuple, z=zval, w=wval)
    )
    @test traj.bounds.x == xtuple
    @test traj.bounds.y == ([ylow], [yup])
    @test traj.bounds.z == (fill(-zval, traj.dims.z), fill(zval, traj.dims.z))
    @test traj.bounds.w == (-wval, wval)
    @test :Δt ∉ traj.bounds
end

end
