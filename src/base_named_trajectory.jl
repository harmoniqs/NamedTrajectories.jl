module BaseNamedTrajectory

using TestItems
using LazyArrays

using ..StructNamedTrajectory
using ..StructKnotPoint


function Base.show(io::IO, Z::NamedTrajectory)
    @inline function format(name, inds)
        str = name == Z.timestep ? "→ " * String(name) : String(name)
        return "$(str) = $(inds)"
    end

    comp_str = join([format(n, Z.components[n]) for n in keys(Z.components)], ", ")
    if isempty(Z.global_data)
        print(io, "N = ", Z.N, ", (", comp_str, ")")
    else
        global_comp_str = join([format(n, Z.global_components[n]) for n in keys(Z.global_components)], ", ")
        print(io, "N = ", Z.N, ", (", comp_str, "), (", global_comp_str, ")")
    end
end

"""
    length(Z::NamedTrajectory) = Z.dim * Z.N + Z.global_dim
"""
function Base.length(Z::NamedTrajectory)
    return Z.dim * Z.N + Z.global_dim
end

"""
    vec(Z::NamedTrajectory) = vcat(Z.datavec, Z.global_data)

"""
function Base.vec(Z::NamedTrajectory)
    # Allocation-free concatenation with LazyArrays
    return ApplyArray(vcat, Z.datavec, Z.global_data)
end

"""
    size(Z::NamedTrajectory) = (dim = Z.dim, N = Z.N, global_dim = Z.global_dim)
"""
Base.size(Z::NamedTrajectory) = (dim = Z.dim, N = Z.N, global_dim = Z.global_dim)

"""
    copy(::NamedTrajectory)

Returns a shallow copy of the trajectory.
"""
function Base.copy(traj::NamedTrajectory)
    NamedTrajectory(
        traj.datavec,
        traj.N,
        traj.timestep,
        traj.dim,
        traj.dims,
        traj.bounds,
        traj.initial,
        traj.final,
        traj.goal,
        traj.components,
        traj.names,
        traj.state_names,
        traj.control_names,
        traj.global_data,
        traj.global_dim,
        traj.global_dims,
        traj.global_components,
        traj.global_names,
    )
end

# -------------------------------------------------------------- #
# Base indexing
# -------------------------------------------------------------- #

"""
    KnotPoint(Z::NamedTrajectory, k::Int)

    # Arguments
    - `Z::NamedTrajectory`: The trajectory from which the KnotPoint is taken.
    - `k::Int`: The timestep of the KnotPoint.
"""
function StructKnotPoint.KnotPoint(
    Z::NamedTrajectory,
    k::Int
)
    @assert 1 ≤ k ≤ Z.N
    timestep = Z[Z.timestep][k]
    return KnotPoint(k, view(Z.data, :, k), timestep, Z.components, Z.names, Z.control_names)
end

"""
    getindex(traj, k::Int)::KnotPoint

Returns the knot point at time `k`.
"""
Base.getindex(traj::NamedTrajectory, k::Int) = KnotPoint(traj, k)

"""
    getindex(traj, ks::AbstractVector{Int})::Vector{KnotPoint}

Returns the knot points at times `ks`.
"""
function Base.getindex(traj::NamedTrajectory, ks::AbstractVector{Int})::Vector{KnotPoint}
    return [traj[k] for k ∈ ks]
end

"""
    lastindex(traj::NamedTrajectory)

Returns the final time index of the trajectory.
"""
Base.lastindex(traj::NamedTrajectory) = traj.N

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
        return reshape(view(traj.datavec, :), :, traj.N)
    elseif symb ∈ fieldnames(NamedTrajectory)
        return getfield(traj, symb)
    elseif symb in traj.names
        indices = traj.components[symb]
        return view(traj.data, indices, :)
    elseif symb in traj.global_names
        indices = traj.global_components[symb]
        return view(traj.global_data, indices)
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
        traj.data[traj.components[symb], :] = val
    end
end

# -------------------------------------------------------------- #
# Equality
# -------------------------------------------------------------- #

"""
    isequal(traj1::NamedTrajectory, traj2::NamedTrajectory)

Check if trajectories are equal w.r.t. data using `Base.isequal`
"""
function Base.isequal(traj1::NamedTrajectory, traj2::NamedTrajectory)
    # check components
    if !issetequal(traj1.names, traj2.names)
        return false
    end

    for name in traj1.names
        if !isequal(traj1[name], traj2[name])
            return false
        end
    end

    # check global components
    if !issetequal(traj1.global_names, traj2.global_names)
        return false
    end

    for gname in traj1.global_names
        if !isequal(traj1[gname], traj2[gname])
            return false
        end
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
    return NamedTrajectory(traj, datavec = α * traj.datavec)
end

Base.:*(traj::NamedTrajectory, α::Float64) = α * NamedTrajectory(traj)

function Base.:+(traj1::NamedTrajectory, traj2::NamedTrajectory)
    @assert traj1.names == traj2.names
    @assert traj1.dim == traj2.dim
    @assert traj1.N == traj2.N
    return NamedTrajectory(traj1, datavec=traj1.datavec + traj2.datavec)
end

function Base.:-(traj1::NamedTrajectory, traj2::NamedTrajectory)
    @assert traj1.names == traj2.names
    @assert traj1.dim == traj2.dim
    @assert traj1.N == traj2.N
    return NamedTrajectory(traj1, datavec=traj1.datavec - traj2.datavec)
end

# =========================================================================== #

@testitem "equality" begin
    using Random
    data = randn(5, 10)
    traj1 = NamedTrajectory(data, (x = 1:3, y=4:4, z=5:5), timestep=:z)
    traj2 = NamedTrajectory(data[[5,4,1,2,3], :], (z=1:1, y=2:2, x=3:5), timestep=:z)
    @test traj1 == traj2

   traj1 = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z,
        global_data=[1.0, 2.0, 3.0], global_components=(a=1:2, b=3:3)
    )
    traj2 = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z,
        global_data=[3.0, 1.0, 2.0], global_components=(a=2:3, b=1:1)
    )
    @test traj1 == traj2
end

@testitem "copy" begin
    using Random
    data1 = randn(5, 10)
    data2 = copy(data1)
    global_data1 = [1.0, 2.0, 3.0]
    global_data2 = copy(global_data1)
    traj1 = NamedTrajectory(
        data1, (x = 1:3, y=4:4, z=5:5), timestep=:z,
        global_data=global_data1, global_components=(a=1:2, b=3:3)
    )
    traj2 = deepcopy(traj1)
    traj1.data .= 0
    @test traj1.data == zeros(size(data1))
    @test traj2.data == data2
    
    traj1.global_data .= 0
    @test traj1.global_data == zeros(size(global_data1))
    @test traj2.global_data == global_data2

end

@testitem "knot point methods" begin
    include("../test/test_utils.jl")
    traj = get_free_time_traj()

    # freetime
    @test traj[1] isa KnotPoint
    @test traj[1].x == traj.x[:, 1]
    @test traj[end] isa KnotPoint
    @test traj[end].x == traj.x[:, end]
    @test traj[:x] == traj.x
    @test traj.timestep isa Symbol
end

@testitem "algebraic methods" begin
    include("../test/test_utils.jl")
    traj = get_free_time_traj()
    traj2 = copy(traj)

    @test (traj + traj2).x == traj.x + traj2.x
    @test (traj - traj2).x == traj.x - traj2.x
    @test (2.0 * traj).x == (traj * 2.0).x == traj.x * 2.0
end

@testitem "copying and equality checks" begin
    include("../test/test_utils.jl")
    traj = get_free_time_traj()

    free_time_traj_copy = copy(traj)
    @test traj == free_time_traj_copy
end

@testitem "length named trajectory" begin
    using Random
    data = randn(5, 10)
    global_data = [1.0, 2.0, 3.0]
    traj = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z,
        global_data=global_data, global_components=(a=1:2, b=3:3)
    )
    @test length(traj) == size(data, 1) * size(data, 2) + length(global_data)
end

@testitem "vec named trajectory" begin
    using Random
    data = randn(5, 10)
    global_data = [1.0, 2.0, 3.0]
    traj = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z,
        global_data=global_data, global_components=(a=1:2, b=3:3)
    )
    
    @test vec(traj) == vcat(traj.datavec, traj.global_data)
end

end