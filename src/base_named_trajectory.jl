module BaseNamedTrajectory

using TestItems

using ..StructNamedTrajectory
using ..StructKnotPoint


function Base.show(io::IO, Z::NamedTrajectory)
    if isempty(Z.gdata)
        print(io, Z.components, ", T = ", Z.T)
    else
        print(io, Z.components, ", T = ", Z.T, ", ", Z.gcomponents)
    end
end

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

"""
    copy(::NamedTrajectory)

Returns a copy of the trajectory data and global data.
"""
function Base.copy(traj::NamedTrajectory)
    return NamedTrajectory(deepcopy(traj.datavec), traj, gdata=deepcopy(traj.gdata))
end

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
    timestep = Z[Z.timestep][t]
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
        indices = traj.gcomponents[symb]
        return view(traj.gdata, indices)
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
    if !issetequal(traj1.gnames, traj2.gnames)
        return false
    end

    for gname in traj1.gnames
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

# =========================================================================== #

@testitem "equality" begin
    using Random
    data = randn(5, 10)
    traj1 = NamedTrajectory(data, (x = 1:3, y=4:4, z=5:5), timestep=:z)
    traj2 = NamedTrajectory(data[[5,4,1,2,3], :], (z=1:1, y=2:2, x=3:5), timestep=:z)
    @test traj1 == traj2

   traj1 = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z,
        gdata=[1.0, 2.0, 3.0], gcomponents=(a=1:2, b=3:3)
    )
    traj2 = NamedTrajectory(
        data, (x = 1:3, y=4:4, z=5:5), timestep=:z,
        gdata=[3.0, 1.0, 2.0], gcomponents=(a=2:3, b=1:1)
    )
    @test traj1 == traj2
end

@testitem "copy" begin
    using Random
    data1 = randn(5, 10)
    data2 = copy(data1)
    gdata1 = [1.0, 2.0, 3.0]
    gdata2 = copy(gdata1)
    traj1 = NamedTrajectory(
        data1, (x = 1:3, y=4:4, z=5:5), timestep=:z,
        gdata=gdata1, gcomponents=(a=1:2, b=3:3)
    )
    traj2 = copy(traj1)
    traj1.data .= 0
    @test traj1.data == zeros(size(data1))
    @test traj2.data == data2
    
    traj1.gdata .= 0
    @test traj1.gdata == zeros(size(gdata1))
    @test traj2.gdata == gdata2

end

@testitem "knot point methods" begin
    include("../test/test_utils.jl")
    free_time_traj = get_free_time_traj()

    # freetime
    @test free_time_traj[1] isa KnotPoint
    @test free_time_traj[1].x == free_time_traj.x[:, 1]
    @test free_time_traj[end] isa KnotPoint
    @test free_time_traj[end].x == free_time_traj.x[:, end]
    @test free_time_traj[:x] == free_time_traj.x
    @test free_time_traj.timestep isa Symbol
end

@testitem "algebraic methods" begin
    include("../test/test_utils.jl")
    free_time_traj = get_free_time_traj()
    free_time_traj2 = copy(free_time_traj)

    @test (free_time_traj + free_time_traj2).x == free_time_traj.x + free_time_traj2.x
    @test (free_time_traj - free_time_traj2).x == free_time_traj.x - free_time_traj2.x
    @test (2.0 * free_time_traj).x == (free_time_traj * 2.0).x == free_time_traj.x * 2.0
end

@testitem "copying and equality checks" begin
    include("../test/test_utils.jl")
    free_time_traj = get_free_time_traj()

    free_time_traj_copy = copy(free_time_traj)
    @test free_time_traj == free_time_traj_copy
end


end