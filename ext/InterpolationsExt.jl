module InterpolationsExt

using NamedTrajectories
using NamedTrajectories.MethodsNamedTrajectory: drop
using DataInterpolations
using TestItems


"""
    trajectory_interpolation(
        traj::NamedTrajectory,
        times::AbstractVector;
        interpolations::NamedTuple=NamedTuple(zip(traj.names, fill(:linear, length(traj.names))))
    )

Interpolate a `NamedTrajectory` at specified time points.

# Arguments
- `traj::NamedTrajectory`: The trajectory to interpolate.
- `times::AbstractVector`: The time points at which to interpolate the trajectory.

# Keyword Arguments
- `interpolations::NamedTuple`: A named tuple specifying the interpolation method for each 
  component. Supported methods are `:constant`, `:linear`, and `:spline`. Defaults to 
  `:linear` for all components.

# Returns
- `NamedTrajectory`: A new trajectory with interpolated values at the specified times.

# Notes
- Components not specified in `interpolations` will be dropped from the returned trajectory.
- The timestep component is automatically included with linear interpolation if not specified.
- Spline interpolation requires derivative components (e.g., `du` for component `u`).

# Examples
```julia
# Linear interpolation at new time points
new_times = [0.0, 0.5, 1.0, 1.5, 2.0]
new_traj = trajectory_interpolation(traj, new_times)

# Mix of interpolation methods
interpolations = (x = :linear, u = :spline, Δt = :linear)
new_traj = trajectory_interpolation(traj, new_times; interpolations=interpolations)
```
"""
function NamedTrajectories.trajectory_interpolation(
    traj::NamedTrajectory,
    times::AbstractVector;
    interpolations::NamedTuple=NamedTuple(
        zip(traj.names, fill(:linear, length(traj.names)))
    ),
)
    if traj.timestep ∉ keys(interpolations)
        interpolations = merge(interpolations, NamedTuple((traj.timestep => :linear,)))
    end

    components = NamedTuple(
        if traj.timestep == name 
            traj.timestep => vcat(diff(times), traj[traj.timestep][:, end])
        elseif kind == :constant
            name => ConstantInterpolation(traj, name)(times)
        elseif kind == :linear
            name => LinearInterpolation(traj, name)(times)
        elseif kind == :spline
            name => CubicHermiteSpline(traj, name)(times)
        else
            throw(ArgumentError("Unsupported interpolation kind: $kind"))
        end
        for (name, kind) in pairs(interpolations)
    )

    # Drop components that are not in the names list
    drop_names = setdiff(traj.names, keys(interpolations))

    # Build new trajectory with interpolated data and new times
    return NamedTrajectory(
        components;
        timestep=traj.timestep,
        controls=drop(traj.control_names, drop_names),
        bounds=drop(traj.bounds, drop_names),
        initial=drop(traj.initial, drop_names),
        final=drop(traj.final, drop_names),
        goal=drop(traj.goal, drop_names),
        global_data=traj.global_data,
        global_components=traj.global_components
    )
end

"""
    trajectory_interpolation(
        traj::NamedTrajectory,
        T::Int;
        kwargs...
    )

Interpolate a `NamedTrajectory` to a new number of time steps.

# Arguments
- `traj::NamedTrajectory`: The trajectory to interpolate.
- `T::Int`: The number of time steps in the interpolated trajectory.

# Keyword Arguments
- `kwargs...`: Additional keyword arguments passed to the main `trajectory_interpolation` method.

# Returns
- `NamedTrajectory`: A new trajectory with `T` time steps, evenly spaced between the original 
  start and end times.

# Examples
```julia
# Interpolate to 100 time steps
new_traj = trajectory_interpolation(traj, 100)

# With custom interpolation methods
new_traj = trajectory_interpolation(traj, 100; interpolations=(x=:spline, u=:linear))
```
"""
function NamedTrajectories.trajectory_interpolation(
    traj::NamedTrajectory,
    T::Int;
    kwargs...
)
    prev_times = get_times(traj)
    new_times = range(prev_times[1], prev_times[end], length=T)
    return trajectory_interpolation(traj, new_times; kwargs...)
end

"""
    ConstantInterpolation(traj::NamedTrajectory, x::Symbol; kwargs...)

Create a constant (zero-order hold) interpolation object for a trajectory component.

# Arguments
- `traj::NamedTrajectory`: The trajectory containing the component to interpolate.
- `x::Symbol`: The name of the component to interpolate.

# Keyword Arguments
- `kwargs...`: Additional keyword arguments passed to `DataInterpolations.ConstantInterpolation`.

# Returns
- `ConstantInterpolation`: An interpolation object that can be called with time values to 
  get interpolated component values.

# Throws
- `AssertionError`: If the component `x` is not found in the trajectory.

# Examples
```julia
# Create an interpolation object
interp = ConstantInterpolation(traj, :x)

# Evaluate at a specific time
value = interp(2.5)
```
"""
function DataInterpolations.ConstantInterpolation(
    traj::NamedTrajectory, x::Symbol; kwargs...
)
    @assert x ∈ traj.names "Component $x not found in trajectory names."
    return ConstantInterpolation(traj[x], get_times(traj); kwargs...)
end

"""
    LinearInterpolation(traj::NamedTrajectory, x::Symbol; kwargs...)

Create a linear (first-order) interpolation object for a trajectory component.

# Arguments
- `traj::NamedTrajectory`: The trajectory containing the component to interpolate.
- `x::Symbol`: The name of the component to interpolate.

# Keyword Arguments
- `kwargs...`: Additional keyword arguments passed to `DataInterpolations.LinearInterpolation`.

# Returns
- `LinearInterpolation`: An interpolation object that can be called with time values to 
  get interpolated component values.

# Throws
- `AssertionError`: If the component `x` is not found in the trajectory.

# Examples
```julia
# Create an interpolation object
interp = LinearInterpolation(traj, :x)

# Evaluate at a specific time
value = interp(2.5)
```
"""
function DataInterpolations.LinearInterpolation(
    traj::NamedTrajectory, x::Symbol; kwargs...
)
    @assert x ∈ traj.names "Component $x not found in trajectory names."
    return LinearInterpolation(traj[x], get_times(traj); kwargs...)
end

"""
    CubicHermiteSpline(traj::NamedTrajectory, dx::Symbol, x::Symbol; kwargs...)

Create a cubic Hermite spline interpolation object for a trajectory component using its derivative.

# Arguments
- `traj::NamedTrajectory`: The trajectory containing the component and its derivative.
- `dx::Symbol`: The name of the derivative component (e.g., `:du` for velocity).
- `x::Symbol`: The name of the component to interpolate (e.g., `:u` for position).

# Keyword Arguments
- `kwargs...`: Additional keyword arguments passed to `DataInterpolations.CubicHermiteSpline`.

# Returns
- `CubicHermiteSpline`: An interpolation object that can be called with time values to 
  get interpolated component values.

# Throws
- `AssertionError`: If either component `x` or derivative component `dx` is not found in the trajectory.

# Examples
```julia
# Create a spline interpolation with explicit derivative component
interp = CubicHermiteSpline(traj, :du, :u)

# Evaluate at a specific time
value = interp(2.5)
```
"""
function DataInterpolations.CubicHermiteSpline(
    traj::NamedTrajectory, dx::Symbol, x::Symbol; kwargs...
)
    @assert x ∈ traj.names "Component $x not found in trajectory names."
    @assert dx ∈ traj.names "Derivative component $dx not found in trajectory names."
    return CubicHermiteSpline(traj[dx], traj[x], get_times(traj); kwargs...)
end

"""
    CubicHermiteSpline(traj::NamedTrajectory, x::Symbol; kwargs...)

Create a cubic Hermite spline interpolation object for a trajectory component with automatic derivative lookup.

# Arguments
- `traj::NamedTrajectory`: The trajectory containing the component and its derivative.
- `x::Symbol`: The name of the component to interpolate (e.g., `:u`).

# Keyword Arguments
- `kwargs...`: Additional keyword arguments passed to `DataInterpolations.CubicHermiteSpline`.

# Returns
- `CubicHermiteSpline`: An interpolation object that can be called with time values to 
  get interpolated component values.

# Notes
- The derivative component is automatically inferred by prepending 'd' to the component name.
  For example, if `x = :u`, the derivative component is assumed to be `:du`.

# Throws
- `AssertionError`: If either component `x` or its automatically inferred derivative component 
  is not found in the trajectory.

# Examples
```julia
# Create a spline interpolation (assumes :du exists in trajectory)
interp = CubicHermiteSpline(traj, :u)

# Evaluate at a specific time
value = interp(2.5)
```
"""
function DataInterpolations.CubicHermiteSpline(
    traj::NamedTrajectory, x::Symbol; kwargs...
)
    dx = Symbol("d" * string(x))
    return CubicHermiteSpline(traj, dx, x; kwargs...)
end

# --------------------------------------------------------------------------- #
# Patches for DataInterpolations.jl
# --------------------------------------------------------------------------- #

# DataInterpolations.jl:src/parameter_caches.jl
function DataInterpolations.cubic_hermite_spline_parameters(du::AbstractArray, u::AbstractArray, t, idx)
    ax_u = axes(u)[1:end-1]
    ax_du = axes(du)[1:end-1]
    Δt = t[idx + 1] - t[idx]
    u₀ = u[ax_u..., idx]
    u₁ = u[ax_u..., idx + 1]
    du₀ = du[ax_du..., idx]
    du₁ = du[ax_du..., idx + 1]
    c₁ = (u₁ - u₀ - du₀ * Δt) / Δt^2
    c₂ = (du₁ - du₀ - 2c₁ * Δt) / Δt^2
    return c₁, c₂
end

# DataInterpolations.jl:src/interpolation_methods.jl
function DataInterpolations._interpolate(
        A::CubicHermiteSpline{<:AbstractArray{<:Number}}, t::Number, iguess)
    ax_u = axes(A.u)[1:end-1]
    ax_du = axes(A.du)[1:end-1]
    idx = DataInterpolations.get_idx(A, t, iguess)
    Δt₀ = t - A.t[idx]
    Δt₁ = t - A.t[idx + 1]
    out = A.u[ax_u..., idx] + Δt₀ * A.du[ax_du..., idx]
    c₁, c₂ = DataInterpolations.get_parameters(A, idx)
    out += Δt₀^2 * (c₁ + Δt₁ * c₂)
    out
end

# *************************************************************************** #

@testitem "test interpolation types" begin
    using DataInterpolations

    T = 10
    x_dim = 3
    u_dim = 2
    traj_init = rand(NamedTrajectory, T, control_dim=u_dim, state_dim=x_dim)
    traj = add_component(traj_init, :du, rand(u_dim, T))

    interp = ConstantInterpolation(traj, :x)
    @test interp isa ConstantInterpolation
    res = interp(2.0)
    @test res isa Vector{Float64}
    @test length(res) == x_dim

    interp = LinearInterpolation(traj, :x)
    @test interp isa LinearInterpolation
    res = interp(2.0)
    @test res isa Vector{Float64}
    @test length(res) == x_dim

    interp = CubicHermiteSpline(traj, :du, :u)
    @test interp isa CubicHermiteSpline
    res = interp(2.0)
    @test res isa Vector{Float64}
    @test length(res) == u_dim

    interp = CubicHermiteSpline(traj, :u)
    @test interp isa CubicHermiteSpline
    res = interp(2.0)
    @test res isa Vector{Float64}
    @test length(res) == u_dim
end

@testitem "interpolation basic functionality" begin
    using DataInterpolations

    # Create a simple test trajectory
    T = 10
    timesteps = fill(1.0, T)
    times = collect(0.0:1.0:T-1)
    data = stack([sin.(times), cos.(times), timesteps], dims=1)
    comps = (x = 1:1, y = 2:2, Δt = 3:3)
    traj = NamedTrajectory(vec(data), comps, T; timestep=:Δt)

    new_times = collect(range(times[1], times[end], length=2T))
    new_traj = trajectory_interpolation(traj, new_times)

    @test size(new_traj.data, 2) == 2T

    # Check that the timestep component is correct
    @test new_traj[:Δt][1:end-1] ≈ diff(new_times)

    # Check that interpolating at original times gives back original data
    orig_traj = trajectory_interpolation(traj, times)
    @test orig_traj[:x] ≈ traj[:x]
    @test orig_traj[:y] ≈ traj[:y]
end

@testitem "interpolation with constant data" begin
    using DataInterpolations

    # Create a constant test trajectory
    T = 10
    timesteps = fill(1.0, T)
    times = collect(0.0:1.0:T-1)
    data = stack([ones(length(times)), zeros(length(times)), timesteps], dims=1)
    comps = (x = 1:1, y = 2:2, Δt = 3:3)
    traj = NamedTrajectory(vec(data), comps, T; timestep=:Δt)

    # Check that constant data is interpolated correctly
    new_times = collect(range(times[1], times[end], length=2T))
    new_traj = trajectory_interpolation(traj, new_times)

    @test size(new_traj.data, 2) == 2T

    # Check that the timestep component is correct
    @test new_traj[:Δt][1:end-1] ≈ diff(new_times)

    # Check that new data matches the constant values
    @test all(new_traj[:x] .== 1.0)
    @test all(new_traj[:y] .== 0.0)
end

end