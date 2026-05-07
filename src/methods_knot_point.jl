module MethodsKnotPoint

using TestItems

using ..StructKnotPoint


"""
    getproperty(slice::KnotPoint, symb::Symbol)

Returns the component of the knot point with name `symb` (as a view) or the property of the knot point with name `symb`.
"""
function Base.getproperty(slice::KnotPoint, symb::Symbol)
    if symb in fieldnames(KnotPoint)
        return getfield(slice, symb)
    else
        indices = slice.components[symb]
        return view(slice.data, indices)
    end
end

function Base.getindex(slice::KnotPoint, symb::Symbol)
    if symb in fieldnames(KnotPoint)
        return getfield(slice, symb)
    else
        return view(slice.data, slice.components[symb])
    end
end

"""
    setproperty!(slice::KnotPoint, symb::Symbol, val::Any)

Update a component of the knot point in-place via `update!`.

Setting a struct field is not supported — `KnotPoint` is immutable and the
data buffer is owned by the parent trajectory. Attempting to assign a struct
field raises an error rather than silently aliasing through `setfield!`.
"""
function Base.setproperty!(slice::KnotPoint, symb::Symbol, val::Any)
    if symb in fieldnames(KnotPoint)
        error(
            "KnotPoint is immutable; cannot set struct field :$symb. " *
            "Use traj[i].name = data to update a trajectory component instead.",
        )
    else
        update!(slice, symb, val)
    end
end

function Base.setindex!(slice::KnotPoint, symb::Symbol, val::Any)
    setproperty!(slice, symb, val)
end


"""
    update!(slice::KnotPoint, symb::Symbol, data::AbstractVector{Float64})

Update a component of the knot point.
"""
function update!(slice::KnotPoint, symb::Symbol, data::AbstractVector{Float64})
    @assert symb in slice.names
    @assert size(data, 1) ==
            (slice.components[symb].stop - slice.components[symb].start + 1)

    Base.getproperty(slice, symb)[:] = data
    return nothing
end

# =========================================================================== #

@testitem "Updating trajectory knot points via view" begin
    traj = rand(NamedTrajectory, 5)

    x_orig = deepcopy(traj.x[:, :])
    u_orig = deepcopy(traj.u[:, :])

    x_new = rand(size(x_orig)...)
    u_new = rand(size(u_orig)...)

    idx = rand(1:traj.N)

    traj[idx].x = deepcopy(x_new[:, idx])
    @test traj.x[:, idx] ==
          traj.data[traj.components.x, idx] ==
          traj[idx].x ==
          x_new[:, idx]

    traj[idx].u = deepcopy(u_new[:, idx])
    @test traj.u[:, idx] ==
          traj.data[traj.components.u, idx] ==
          traj[idx].u ==
          u_new[:, idx]

    traj.data[traj.components.x, idx] = deepcopy(x_orig[:, idx])
    @test traj.x[:, idx] ==
          traj.data[traj.components.x, idx] ==
          traj[idx].x ==
          x_orig[:, idx]

    traj.data[traj.components.u, idx] = deepcopy(u_orig[:, idx])
    @test traj.u[:, idx] ==
          traj.data[traj.components.u, idx] ==
          traj[idx].u ==
          u_orig[:, idx]
end

@testitem "KnotPoint getindex via Symbol" begin
    traj = rand(NamedTrajectory, 6)
    kp = traj[3]

    # Component access via getindex: should return view of slice data.
    x_view = kp[:x]
    @test x_view == traj.x[:, 3]
    @test x_view == kp.x

    # Field access via getindex: returns the underlying field value, not a view.
    @test kp[:components] === kp.components
end

@testitem "KnotPoint setproperty! on a struct field errors" begin
    traj = rand(NamedTrajectory, 4)
    kp = traj[2]
    # KnotPoint is immutable; assigning to a struct field must error.
    @test_throws ErrorException kp.components = kp.components
end

end
