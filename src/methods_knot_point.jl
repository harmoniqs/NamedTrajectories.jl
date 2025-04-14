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

function Base.getindex(slice::KnotPoint, symb::Symbol) # is this method redundant? i.e. should we just dispatch to getproperty?
    if symb in fieldnames(KnotPoint)
        return getfield(slice, symb)
    else
        indices = slice.components[symb]
        return view(slice.data, indices)
    end
end

"""
    setproperty!(slice::KnotPoint, symb::Symbol, val::Any)

Dispatches setting properties of knot points as either setting a component or a property via `update!` or `setfield!`, respectively.
"""
function Base.setproperty!(slice::KnotPoint, symb::Symbol, val::Any)
    if symb in fieldnames(KnotPoint)
        setfield!(slice, symb, val) # will throw error since KnotPoint is an immutable struct
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
    @assert size(data, 1) == (slice.components[symb].stop - slice.components[symb].start + 1)

    Base.getproperty(slice, symb)[:] = data
    return nothing
end


@testitem "Updating trajectory knot points via view" begin
    traj = rand(NamedTrajectory, 5)

    x_orig = deepcopy(traj.x[:, :])
    u_orig = deepcopy(traj.u[:, :])

    x_new = rand(size(x_orig)...)
    u_new = rand(size(u_orig)...)

    idx = rand(1:traj.T)

    traj[idx].x = deepcopy(x_new[:, idx])
    @test traj.x[:, idx] == traj.data[traj.components.x, idx] == traj[idx].x == x_new[:, idx]

    traj[idx].u = deepcopy(u_new[:, idx])
    @test traj.u[:, idx] == traj.data[traj.components.u, idx] == traj[idx].u == u_new[:, idx]

    traj.data[traj.components.x, idx] = deepcopy(x_orig[:, idx])
    @test traj.x[:, idx] == traj.data[traj.components.x, idx] == traj[idx].x == x_orig[:, idx]

    traj.data[traj.components.u, idx] = deepcopy(u_orig[:, idx])
    @test traj.u[:, idx] == traj.data[traj.components.u, idx] == traj[idx].u == u_orig[:, idx]
end

end
