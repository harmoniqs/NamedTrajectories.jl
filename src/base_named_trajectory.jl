module BaseNamedTrajectory

export unpack!
export sync_timesteps!
export get_derived_names

using TestItems
using LazyArrays

using ..StructNamedTrajectory
using ..StructKnotPoint
using ..TimeWarp: AbstractTimeWarp, n_params, warp_params, with_params, deltats_row


function Base.show(io::IO, Z::NamedTrajectory)
    @inline function format(name, inds)
        str = if name == Z.timestep && Z.warp !== nothing
            "⇒ " * String(name)  # derived from the warp — never decision data
        elseif name == Z.timestep
            "→ " * String(name)
        else
            String(name)
        end
        return "$(str) = $(inds)"
    end

    comp_str = join([format(n, Z.components[n]) for n in keys(Z.components)], ", ")
    if isempty(Z.global_data)
        print(io, "N = ", Z.N, ", (", comp_str, ")")
    else
        global_comp_str = join(
            [format(n, Z.global_components[n]) for n in keys(Z.global_components)],
            ", ",
        )
        print(io, "N = ", Z.N, ", (", comp_str, "), (", global_comp_str, ")")
    end
end

"""
    length(Z::NamedTrajectory)

Length of the packed decision vector: all non-derived components per knot, the
global data, and the warp parameters. Without a warp this is the historical
accounting `Z.dim * Z.N + Z.global_dim`; with a warp the derived timestep rows
are excluded and `n_params(Z.warp)` is appended. Always `== length(vec(Z))`.
"""
function Base.length(Z::NamedTrajectory)
    warp = getfield(Z, :warp)
    if warp === nothing
        return Z.dim * Z.N + Z.global_dim
    end
    return (Z.dim - Z.dims[Z.timestep]) * Z.N + Z.global_dim + n_params(warp)
end

"""
    vec(Z::NamedTrajectory)

The packed decision vector. Without a warp this is the historical lazy
concatenation `[Z.datavec; Z.global_data]`. With a warp, the derived timestep
rows are **excluded**: per knot, the non-derived component rows in component
order, then `Z.global_data`, then the trailing warp parameters
(`warp_params(Z.warp)` — for [`GlobalScale`](@ref), the scalar duration `T`).
"""
function Base.vec(Z::NamedTrajectory)
    # Allocation-free concatenation with LazyArrays
    warp = getfield(Z, :warp)
    if warp === nothing
        return ApplyArray(vcat, Z.datavec, Z.global_data)
    end
    # packed layout: drop the derived timestep row per knot; globals; warp params
    zdim = Z.dim - Z.dims[Z.timestep]
    dt_idx = first(Z.components[Z.timestep])
    out = Vector{eltype(Z.datavec)}(undef, length(Z))
    for k = 1:Z.N
        block = view(Z.data, :, k)
        copyto!(out, (k-1)*zdim + 1, block, 1, dt_idx - 1)
        copyto!(out, (k-1)*zdim + dt_idx, block, dt_idx + 1, Z.dim - dt_idx)
    end
    copyto!(out, zdim*Z.N + 1, Z.global_data, 1, Z.global_dim)
    copyto!(out, zdim*Z.N + Z.global_dim + 1, warp_params(warp), 1, n_params(warp))
    return out
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
        traj.warp,
    )
end

# -------------------------------------------------------------- #
# Derived timesteps — the packed write path
# -------------------------------------------------------------- #

"""
    get_derived_names(traj::NamedTrajectory)::Tuple{Vararg{Symbol}}

Names of the DERIVED components — those present as rows but excluded from the
packed decision vector. This is decided by the component's ROLE (the timestep of
a warped trajectory), never by its name: a component named `:Δt` that is not the
timestep is not derived, and the timestep is derived under a warp whatever its name.
Returns `()` without a warp.
"""
function get_derived_names(traj::NamedTrajectory)
    warp = getfield(traj, :warp)
    return warp === nothing ? () : (traj.timestep,)
end

"""
    sync_timesteps!(traj::NamedTrajectory)

Rewrite the derived timestep rows from `(warp, N)` via [`deltats_row`](@ref). This is
the ONLY writer of the derived rows — everything else goes through construction or
[`unpack!`](@ref). No-op (returns `traj`) without a warp.
"""
function sync_timesteps!(traj::NamedTrajectory)
    warp = getfield(traj, :warp)
    warp === nothing && return traj
    data = traj.data
    data[traj.components[traj.timestep], :] .= reshape(deltats_row(warp, traj.N), 1, :)
    return traj
end

"""
    unpack!(traj::NamedTrajectory, z::AbstractVector{<:Real})

Write the packed decision vector `z` — the layout of `vec(traj)` — into `traj`:

  - the non-derived components only: under a warp the derived timestep rows are
    NEVER written from `z`;
  - the trailing warp-parameter block rebuilds `traj.warp` via [`with_params`](@ref)
    (when the warp carries parameters);
  - then [`sync_timesteps!`](@ref) re-derives the timestep rows.

Throws `DimensionMismatch` when `length(z) != length(traj)`. Returns `traj`.
"""
function unpack!(traj::NamedTrajectory, z::AbstractVector{<:Real})
    length(z) == length(traj) || throw(
        DimensionMismatch(
            "length(z) = $(length(z)) does not match the packed length $(length(traj))",
        ),
    )
    warp = getfield(traj, :warp)
    if warp === nothing
        # nothing is derived: z is [datavec; global_data]
        copyto!(traj.datavec, view(z, 1:(traj.dim*traj.N)))
        copyto!(traj.global_data, view(z, (traj.dim*traj.N+1):length(z)))
    else
        _unpack_derived!(traj, warp, z)
    end
    sync_timesteps!(traj)
    return traj
end

function _unpack_derived!(
    traj::NamedTrajectory,
    warp::AbstractTimeWarp,
    z::AbstractVector{<:Real},
)
    zdim = traj.dim - traj.dims[traj.timestep]
    dt_idx = first(traj.components[traj.timestep])
    data = traj.data
    for k = 1:traj.N
        dst = view(data, :, k)
        src0 = (k - 1) * zdim
        copyto!(dst, 1, z, src0 + 1, dt_idx - 1)
        copyto!(dst, dt_idx + 1, z, src0 + dt_idx, traj.dim - dt_idx)
    end
    # trailing warp parameters rebuild the warp
    n = n_params(warp)
    if n > 0
        z0 = zdim * traj.N + traj.global_dim
        traj.warp = with_params(warp, Vector{Float64}(view(z, (z0+1):(z0+n))))
    end
    return traj
end

# -------------------------------------------------------------- #
# Base indexing
# -------------------------------------------------------------- #

"""
    KnotPoint(Z::NamedTrajectory, k::Int)

    # Arguments
    - `Z::NamedTrajectory`: The trajectory from which the KnotPoint is taken.
    - `k::Int`: The index of the knot point.
"""
function StructKnotPoint.KnotPoint(Z::NamedTrajectory, k::Int)
    @assert 1 ≤ k ≤ Z.N
    timestep = Z[Z.timestep][k]
    return KnotPoint(
        k,
        view(Z.data, :, k),
        timestep,
        Z.components,
        Z.names,
        Z.control_names,
    )
end

"""
    getindex(traj, k::Int)::KnotPoint

Returns the knot point at index `k`.
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
    return NamedTrajectory(traj1, datavec = traj1.datavec + traj2.datavec)
end

function Base.:-(traj1::NamedTrajectory, traj2::NamedTrajectory)
    @assert traj1.names == traj2.names
    @assert traj1.dim == traj2.dim
    @assert traj1.N == traj2.N
    return NamedTrajectory(traj1, datavec = traj1.datavec - traj2.datavec)
end

# =========================================================================== #

@testitem "equality" begin
    using Random
    data = randn(5, 10)
    traj1 = NamedTrajectory(data, (x = 1:3, y = 4:4, z = 5:5), timestep = :z)
    traj2 = NamedTrajectory(
        data[[5, 4, 1, 2, 3], :],
        (z = 1:1, y = 2:2, x = 3:5),
        timestep = :z,
    )
    @test traj1 == traj2

    traj1 = NamedTrajectory(
        data,
        (x = 1:3, y = 4:4, z = 5:5),
        timestep = :z,
        global_data = [1.0, 2.0, 3.0],
        global_components = (a = 1:2, b = 3:3),
    )
    traj2 = NamedTrajectory(
        data,
        (x = 1:3, y = 4:4, z = 5:5),
        timestep = :z,
        global_data = [3.0, 1.0, 2.0],
        global_components = (a = 2:3, b = 1:1),
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
        data1,
        (x = 1:3, y = 4:4, z = 5:5),
        timestep = :z,
        global_data = global_data1,
        global_components = (a = 1:2, b = 3:3),
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
        data,
        (x = 1:3, y = 4:4, z = 5:5),
        timestep = :z,
        global_data = global_data,
        global_components = (a = 1:2, b = 3:3),
    )
    @test length(traj) == size(data, 1) * size(data, 2) + length(global_data)
end

@testitem "vec named trajectory" begin
    using Random
    data = randn(5, 10)
    global_data = [1.0, 2.0, 3.0]
    traj = NamedTrajectory(
        data,
        (x = 1:3, y = 4:4, z = 5:5),
        timestep = :z,
        global_data = global_data,
        global_components = (a = 1:2, b = 3:3),
    )

    @test vec(traj) == vcat(traj.datavec, traj.global_data)
end

@testitem "size returns (dim, N, global_dim)" begin
    traj = NamedTrajectory(
        randn(5, 10),
        (x = 1:3, y = 4:4, z = 5:5);
        timestep = :z,
        global_data = [1.0, 2.0, 3.0],
        global_components = (a = 1:2, b = 3:3),
    )
    sz = size(traj)
    @test sz.dim == 5
    @test sz.N == 10
    @test sz.global_dim == 3
end

@testitem "show prints components and global components" begin
    traj_no_global = NamedTrajectory(randn(3, 4), (x = 1:2, z = 3:3); timestep = :z)
    str_no_global = sprint(show, traj_no_global)
    @test occursin("N = 4", str_no_global)
    @test occursin("→ z", str_no_global)
    @test occursin("x", str_no_global)
    @test !occursin("),  (", str_no_global)

    traj_global = NamedTrajectory(
        randn(3, 4),
        (x = 1:2, z = 3:3);
        timestep = :z,
        global_data = [1.0, 2.0],
        global_components = (g = 1:2,),
    )
    str_global = sprint(show, traj_global)
    @test occursin("N = 4", str_global)
    @test occursin("g = 1:2", str_global)
end

@testitem "getindex with vector of integers returns vector of knot points" begin
    traj = rand(NamedTrajectory, 6)
    knots = traj[2:4]
    @test knots isa Vector
    @test length(knots) == 3
    @test knots[1].x == traj[2].x
    @test knots[end].x == traj[4].x
end

@testitem "packed vec and length exclude the derived timestep rows" begin
    using NamedTrajectories

    N = 10
    w = GlobalScale(5.0)
    traj = NamedTrajectory(
        (x = randn(3, N), u = randn(2, N), Δt = fill(0.1, 1, N));
        controls = :u,
        warp = w,
        global_data = [1.0, 2.0],
        global_components = (g = 1:2,),
    )

    z = vec(traj)

    # dimension accounting: non-derived components per knot, globals, warp params
    @test length(traj) == length(z) == (traj.dim - 1) * N + 2 + n_params(w)

    # layout: per-knot non-derived rows in component order, then globals, then params
    expected = Vector{Float64}(undef, 5 * N + 2 + 1)
    for k = 1:N
        expected[((k-1)*5+1):((k-1)*5+3)] = traj.x[:, k]
        expected[((k-1)*5+4):((k-1)*5+5)] = traj.u[:, k]
    end
    expected[(5*N+1):(5*N+2)] = [1.0, 2.0]
    expected[end] = 5.0
    @test z == expected

    # no-warp invariant: length(traj) == length(vec(traj)) still holds
    plain = NamedTrajectory(
        (x = randn(3, N), u = randn(2, N), Δt = fill(0.1, 1, N));
        controls = :u,
        global_data = [1.0, 2.0],
        global_components = (g = 1:2,),
    )
    @test length(plain) == length(vec(plain)) == plain.dim * N + 2
end

@testitem "unpack! round-trips the packed vector and syncs the warp" begin
    using NamedTrajectories

    N = 10
    w = GlobalScale(5.0)
    traj = NamedTrajectory(
        (x = randn(3, N), u = randn(2, N), Δt = fill(0.1, 1, N));
        controls = :u,
        warp = w,
    )

    z = vec(traj)
    before = collect(traj.datavec)

    # identity round trip: unpacking the packed vector changes nothing
    @test unpack!(traj, z) === traj
    @test traj.datavec == before
    @test traj.Δt == reshape(deltats_row(w, N), 1, N)

    # perturbed components and a perturbed T: unpack! writes the components,
    # rebuilds the warp from the trailing params, and re-derives the Δt rows
    z2 = copy(z)
    z2[1] += 0.5
    z2[4] -= 1.0
    z2[end] = 7.5
    unpack!(traj, z2)
    @test traj.x[1, 1] == z2[1]
    @test traj.u[1, 1] == z2[4]
    @test traj.warp == GlobalScale(7.5)
    @test traj.Δt == reshape(deltats_row(GlobalScale(7.5), N), 1, N)
    @test vec(traj) == z2
end

@testitem "unpack! rejects a wrong-length packed vector" begin
    using NamedTrajectories

    traj = NamedTrajectory(
        (x = randn(3, 5), u = randn(2, 5), Δt = fill(0.1, 1, 5));
        controls = :u,
        warp = GlobalScale(5.0),
    )
    @test_throws DimensionMismatch unpack!(traj, vec(traj)[1:(end-1)])
    @test_throws DimensionMismatch unpack!(traj, vcat(vec(traj), 1.0))
end

@testitem "copy carries the warp" begin
    using NamedTrajectories

    traj = NamedTrajectory(
        (x = randn(3, 5), u = randn(2, 5), Δt = fill(0.1, 1, 5));
        controls = :u,
        warp = GlobalScale(5.0),
        global_data = [1.0],
        global_components = (g = 1:1,),
    )
    c = copy(traj)
    @test c.warp === traj.warp
    @test vec(c) == vec(traj)
    @test length(c) == length(traj)
end

@testitem "get_derived_names: role-based, not name-based" begin
    using NamedTrajectories

    plain = NamedTrajectory((x = randn(3, 5), z = fill(0.1, 1, 5)); timestep = :z)
    @test get_derived_names(plain) == ()

    warped = NamedTrajectory(
        (x = randn(3, 5), z = fill(0.1, 1, 5), Δt = fill(0.2, 1, 5));
        timestep = :z,
        warp = GlobalScale(5.0),
    )
    # the TIMESTEP is derived under a warp — not "the component named Δt"
    @test get_derived_names(warped) == (:z,)
end

@testitem "sync_timesteps! is the single writer of the derived rows" begin
    using NamedTrajectories

    N = 10
    w = GlobalScale(5.0)
    traj = NamedTrajectory(
        (x = randn(3, N), u = randn(2, N), Δt = fill(0.1, 1, N));
        controls = :u,
        warp = w,
    )

    # scribble over the derived rows, then sync restores them exactly
    traj.data[traj.components.Δt, :] .= 999.0
    @test sync_timesteps!(traj) === traj
    @test traj.Δt == reshape(deltats_row(w, N), 1, N)

    # no-op (returns the trajectory) without a warp
    plain = NamedTrajectory(
        (x = randn(3, N), u = randn(2, N), Δt = fill(0.1, 1, N));
        controls = :u,
    )
    @test sync_timesteps!(plain) === plain
    @test plain.Δt == fill(0.1, 1, N)
end

@testitem "isequal mismatch branches" begin
    base = NamedTrajectory(
        randn(4, 5),
        (x = 1:2, y = 3:3, z = 4:4);
        timestep = :z,
        global_data = [1.0, 2.0],
        global_components = (g = 1:2,),
    )

    # Mismatched component names: rebuild with a different component name set.
    differ_names = NamedTrajectory(
        randn(4, 5),
        (x = 1:2, w = 3:3, z = 4:4);
        timestep = :z,
        global_data = [1.0, 2.0],
        global_components = (g = 1:2,),
    )
    @test !isequal(base, differ_names)

    # Same names, different component data.
    differ_values = NamedTrajectory(
        randn(4, 5),
        (x = 1:2, y = 3:3, z = 4:4);
        timestep = :z,
        global_data = [1.0, 2.0],
        global_components = (g = 1:2,),
    )
    @test !isequal(base, differ_values)

    # Mismatched global names.
    differ_globals = NamedTrajectory(
        copy(base.data),
        (x = 1:2, y = 3:3, z = 4:4);
        timestep = :z,
        global_data = [1.0, 2.0],
        global_components = (h = 1:2,),
    )
    @test !isequal(base, differ_globals)

    # Same global names, different global values.
    differ_global_values = NamedTrajectory(
        copy(base.data),
        (x = 1:2, y = 3:3, z = 4:4);
        timestep = :z,
        global_data = [3.0, 4.0],
        global_components = (g = 1:2,),
    )
    @test !isequal(base, differ_global_values)
end

end
