module StructNamedTrajectory

export NamedTrajectory

using OrderedCollections
using TestItems

# ---------------------------------------------------------------------------- #
# Types for NamedTrajectory
# ---------------------------------------------------------------------------- #

const DimType = Tuple{Vararg{Int}}

const NameType = Tuple{Vararg{Symbol}}

# UnitRange{Int} enforces nonallocating @views with indexing
const ComponentType = Tuple{Vararg{UnitRange{Int}}}

const DataType{R <: Real} = Tuple{Vararg{R⃗}} where R⃗ <: AbstractVector{R}

# Bound type is the type stored by the trajectory
const BoundType{R <: Real} = Tuple{Vararg{Tuple{R⃗, R⃗}}} where R⃗ <: AbstractVector{R}

# Allowed bounds at construction (elements of `bounds` NamedTuple)
const ScalarBound = Union{R, Tuple{R, R}} where R <: Real
const VectorBound = Union{R⃗, Tuple{R⃗, R⃗}} where R⃗ <: AbstractVector{<:Real}
const AbstractBound = Union{ScalarBound, VectorBound}

# ---------------------------------------------------------------------------- #
# Named Trajectory
# ---------------------------------------------------------------------------- #

"""
    NamedTrajectory{R <: Real}

Container for trajectory optimization problems, which includes the trajectory data, bounds dimensions, initial and final conditions, goal states, and components.

This struct is designed to hold trajectory data in a named format, allowing for easy access to knot points by `Symbol`.

NamedTrajectory is designed to make allocation-free access easy to write. The data can be updated after construction, but the fields cannot.
"""
mutable struct NamedTrajectory{
    R <: Real,
    DNames, DTypes <: DimType,
    BNames, BTypes <: BoundType{R},
    INames, ITypes <: DataType{R},
    FNames, FTypes <: DataType{R},
    global_names, GTypes <: DataType{R},
    CNames, CTypes <: ComponentType,
    N <: NameType,
    SN <: NameType,
    CN <: NameType,
    GDNames, GDTypes <: DimType,
    GCNames, GCTypes <: ComponentType,
    GN <: NameType,
}
    datavec::Vector{R}
    N::Int
    timestep::Symbol
    dim::Int
    dims::NamedTuple{DNames, DTypes}
    bounds::NamedTuple{BNames, BTypes}
    initial::NamedTuple{INames, ITypes}
    final::NamedTuple{FNames, FTypes}
    goal::NamedTuple{global_names, GTypes}
    components::NamedTuple{CNames, CTypes}
    names::N
    state_names::SN
    control_names::CN
    global_data::Vector{R}
    global_dim::Int
    global_dims::NamedTuple{GDNames, GDTypes}
    global_components::NamedTuple{GCNames, GCTypes}
    global_names::GN
end

"""
    NamedTrajectory(datavec, components, N)

Construct a named trajectory from a data vector, components, and knot points.
"""
function NamedTrajectory(
    datavec::AbstractVector{R},
    comps::NamedTuple{N, <:ComponentType} where N,
    N::Int;
    timestep::Symbol=:Δt,
    controls::Union{Symbol, NameType}=timestep,
    bounds=NamedTuple(),
    initial=NamedTuple(),
    final=NamedTuple(),
    goal=NamedTuple(),
    global_data::AbstractVector{R}=R[],
    global_components::NamedTuple{GN, <:ComponentType} where GN=NamedTuple(),
) where R <: Real
    @assert :data ∉ keys(comps) "data is a reserved name"
    @assert isdisjoint(keys(comps), keys(global_components)) "components and global components should use unique names"

    @assert timestep isa Symbol && timestep ∈ keys(comps) "Missing timestep in components"

    controls = controls isa Symbol ? (controls,) : controls

    # timestep is a control
    if timestep isa Symbol && !in(timestep, controls)
        controls = (controls..., timestep)
    end
    
    names = Tuple(keys(comps))
    inspect_names(names, controls, keys(initial), keys(final), keys(goal), keys(bounds))
    states = Tuple(k for k ∈ names if k ∉ controls)

    # save dims
    dims_pairs = [(k => length(v)) for (k, v) ∈ pairs(comps)]
    dims = NamedTuple(dims_pairs)
    dim = sum(values(dims), init=0)
    @assert dim * N == length(datavec) "Data vector length does not match components"

    # process and save bounds
    bounds = get_bounds_from_dims(bounds, dims, dtype=R)

    # check data
    inspect_dims_pairs(dims_pairs, bounds, initial, final, goal)

    # global data 
    global_dim = length(global_data)
    global_dims = NamedTuple([(k => length(v)) for (k, v) ∈ pairs(global_components)])
    global_names = Tuple(keys(global_components))

    # check global data
    @assert global_dim == sum(values(global_dims), init=0) "invalid global data dims"

    return NamedTrajectory(
        datavec,
        N,
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
        global_data,
        global_dim,
        global_dims,
        global_components,
        global_names
    )
end

"""
    NamedTrajectory(component_data, global_components_data)

Construct a `NamedTrajectory` from component data and global component data
"""
function NamedTrajectory(
    comps_data::NamedTuple{N, <:Tuple{Vararg{AbstractMatrix{<:Real}}}} where N,
    gcomps_data::NamedTuple{GN, <:Tuple{Vararg{AbstractVector{<:Real}}}} where GN;
    kwargs...
)
    # unpack data (promote type)
    data = vcat([val for (key, val) ∈ pairs(comps_data)]...)
    dim, N = size(data)

    # save components of data matrix
    dims_pairs = [(k => size(v, 1)) for (k, v) ∈ pairs(comps_data)]
    comps_pairs = [(dims_pairs[1][1] => 1:dims_pairs[1][2])]
    for (k, dim) in dims_pairs[2:end]
        k_range = comps_pairs[end][2][end] .+ (1:dim)
        push!(comps_pairs, k => k_range)
    end
    comps = NamedTuple(comps_pairs)

    # save global componets
    if !isempty(gcomps_data)
        global_data = vcat([val for (key, val) ∈ pairs(gcomps_data)]...)
        global_dims_pairs = [(k => length(v)) for (k, v) ∈ pairs(gcomps_data)]
        gcomps_pairs = [(global_dims_pairs[1][1] => 1:global_dims_pairs[1][2])]
        for (k, v) ∈ global_dims_pairs[2:end]
            # offset within global_data
            k_value = gcomps_pairs[end][2][end] .+ (1:v)
            push!(gcomps_pairs, k => k_value)
        end
        gcomps = NamedTuple(gcomps_pairs)
        R = promote_type(eltype(data), eltype(global_data))

        return NamedTrajectory(
            vec(convert.(R, data)), comps, N; 
            global_data=convert.(R, global_data), 
            global_components=gcomps,
            kwargs...
        )
    else
        # user can specify global data using `global_data`, `global_components`
        return NamedTrajectory(vec(data), comps, N; kwargs...)
    end
end

"""
    NamedTrajectory(component_data)

Construct a `NamedTrajectory` from component data.
"""
function NamedTrajectory(
    comps_data::NamedTuple{N, <:Tuple{Vararg{AbstractMatrix{<:Real}}}} where N;
    kwargs...
)
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
    comps_data::NamedTuple{N, <:Tuple{Vararg{AbstractVecOrMat{<:Real}}}} where N; 
    kwargs...
) # where R <: Real
    vals = [v isa AbstractVector ? reshape(v, 1, :) : v for v ∈ values(comps_data)]
    comps_data = NamedTuple([(k => v) for (k, v) ∈ zip(keys(comps_data), vals)])
    return NamedTrajectory(comps_data; kwargs...)
end

"""
    NamedTrajectory(data, traj)

Construct a `NamedTrajectory` from an existing `NamedTrajectory`.
"""
function NamedTrajectory(
    traj::NamedTrajectory;
    datavec::AbstractVector{R}=traj.datavec,
    components::NamedTuple{N, <:ComponentType} where N=traj.components,
    N::Int=traj.N,
    timestep::Symbol=traj.timestep,
    controls::Union{Symbol, NameType}=traj.control_names,
    bounds=traj.bounds,
    initial=traj.initial,
    final=traj.final,
    goal=traj.goal,
    global_data::AbstractVector{R}=traj.global_data,
    global_components::NamedTuple{GN, <:ComponentType} where GN=traj.global_components,
) where R <: Real
    @assert length(datavec) == sum(length.(values(components)), init=0) * N "Data vector length does not match components * N"
    @assert length(global_data) == sum(length.(values(global_components)), init=0) "Global data length does not match global components"

    return NamedTrajectory(
        datavec,
        components,
        N;
        timestep=timestep,
        controls=controls,
        bounds=bounds,
        initial=initial,
        final=final,
        goal=goal,
        global_data=global_data,
        global_components=global_components,
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
    N = size(data, 2)
    datavec = vec(data)
    return NamedTrajectory(datavec, comps, N; kwargs...)
end

# ----------------------------------------------------------------------------- #
# Named trajectory data processing
# ----------------------------------------------------------------------------- #

"""
    get_bounds_from_dims(bounds, dims; dtype=Float64)

Process `bounds` from allowed types using `dims` and convert to `dtype`.
"""
function get_bounds_from_dims(
    bounds::NamedTuple,
    dims::NamedTuple{<:Any, <:DimType};
    dtype=Float64
)   
    bounds_dict = OrderedDict{Symbol, AbstractBound}(pairs(bounds))
    for (name, bound) ∈ bounds_dict
        @assert bound isa AbstractBound

        bdim = dims[name]
        if bound isa Real
            vbound = fill(convert(dtype, bound), bdim)
            bounds_dict[name] = (-vbound, vbound)
        elseif bound isa Tuple{<:Real, <:Real}
            bounds_dict[name] = (
                fill(convert(dtype, bound[1]), bdim), 
                fill(convert(dtype, bound[2]), bdim)
            )
        elseif bound isa AbstractVector{<:Real}
            if length(bound) != bdim 
                throw(ArgumentError("Invalid bound $name: $(length(bound)) != $bdim"))
            end
            bounds_dict[name] = (-convert.(dtype, bound), convert.(dtype, bound))
        elseif bound isa Tuple{<:AbstractVector{<:Real}, <:AbstractVector{<:Real}}
            if length(bound[1]) != bdim 
                throw(ArgumentError("Invalid bound $name: $(length(bound[1])) != $bdim"))
            end
            if length(bound[2]) != bdim 
                throw(ArgumentError("Invalid bound $name: $(length(bound[2])) != $bdim"))
            end
            bounds_dict[name] = (convert.(dtype, bound[1]), convert.(dtype, bound[2]))
        else
            throw(ArgumentError("Unimplemented bound type for $name: $(typeof(bound))"))
        end
    end
    return NamedTuple(bounds_dict)
end

"""
    inspect_names(names, controls, initial, final, goal, bounds)

Check for missing names in the trajectory components.
"""
function inspect_names(
    names::NameType,
    controls::NameType,
    initial::NameType,
    final::NameType,
    goal::NameType,
    bounds::NameType,
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
    bounds::NamedTuple{bname, <:BoundType{R}} where bname,
    initial::NamedTuple{iname, <:DataType{R}} where iname,
    final::NamedTuple{fname, <:DataType{R}} where fname,
    goal::NamedTuple{gname, <:DataType{R}} where gname
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
    N = 10
    data = randn(n, N)
    traj = NamedTrajectory(data, (x = 1:3, y=4:4, Δt=5:5))
    @test traj.data ≈ data
    @test traj.timestep == :Δt
    @test traj.dim == n
    @test traj.N == N
    @test traj.names == (:x, :y, :Δt)

    traj = NamedTrajectory(data, (x = 1:3, y=4:4, z=5:5), timestep=:z)
    @test traj.data ≈ data
    @test traj.timestep == :z
    @test traj.dim == n
    @test traj.N == N
    @test traj.names == (:x, :y, :z)

end

@testitem "Construct from component data" begin
    # define number of timesteps and timestep
    N = 10
    dt = 0.1

    dim = 6
    comps_data = (
        x = rand(3, N),
        u = rand(2, N),
        Δt = fill(dt, 1, N),
    )

    timestep = :Δt
    control = :u

    # some global params as a NamedTuple
    global_dim = 2
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

    @test traj.N == N
    @test traj.dim == dim
    @test length(traj.global_data) == global_dim
    @test traj.names == (:x, :u, :Δt)
    @test traj.state_names == (:x,)
    @test traj.control_names == (:u, :Δt)
    @test traj.global_names == (:α, :β)

    comps_res = NamedTuple([(k => traj.data[v, :]) for (k, v) in pairs(traj.components)])
    @test comps_res == comps_data

    gres = NamedTuple([(k => traj.global_data[v]) for (k, v) in pairs(traj.global_components)])
    @test gres == gcomps_data

    # ignore global
    # ---
    traj = NamedTrajectory(comps_data, controls=control)

    @test traj.N == N
    @test traj.dim == dim
    @test traj.names == (:x, :u, :Δt)
    @test traj.state_names == (:x,)
    @test traj.control_names == (:u, :Δt)
    @test isempty(traj.global_names)

    comps_res = NamedTuple([(k => traj.data[v, :]) for (k, v) in pairs(traj.components)]) 
    @test comps_res == comps_data

    @test isempty(traj.global_data)
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
