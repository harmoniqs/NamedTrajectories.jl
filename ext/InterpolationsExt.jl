module InterpolationsExt

using NamedTrajectories
using Interpolations
using TestItems


# This checks compatability with Interpolations

function trajectory_interpolation(
    times::AbstractVector,
    traj::NamedTrajectory,
    kind::Symbol;
    kwargs...
)
    new_data = zeros(eltype(traj.data), size(traj.data, 1), length(times))

    for name in traj.names
        # Set the timestep component using new times
        if name == traj.timestep
            # last timestep chosen to match the original trajectory
            timesteps = vcat(diff(times), traj[traj.timestep][:, end])
            new_data[traj.components[name], :] .= reshape(timesteps, 1, length(timesteps))
        else
            # interpolate all other components
            vals = traj[name]
            for (i, idx) in enumerate(traj.components[name])
                itp = if kind == :linear
                    linear_interpolation(get_times(traj), vals[i, :]; kwargs...)
                elseif kind == :constant
                    constant_interpolation(get_times(traj), vals[i, :]; kwargs...)
                else
                    throw(ValueError("Unsupported interpolation kind: $kind"))
                end
                new_data[idx, :] = itp(times)
            end
        end
    end

    # Build new trajectory with interpolated data and new times
    return NamedTrajectory(
        vec(new_data),
        traj.components,
        length(times);
        timestep=traj.timestep,
        controls=traj.control_names,
        bounds=traj.bounds,
        initial=traj.initial,
        final=traj.final,
        goal=traj.goal,
        global_data=traj.global_data,
        global_components=traj.global_components
    )
end

function trajectory_interpolation(T::Int, traj::NamedTrajectory, kind::Symbol; kwargs...)
    prev_times = get_times(traj)
    new_times = range(prev_times[1], prev_times[end], length=T)
    return trajectory_interpolation(new_times, traj, kind; kwargs...)
end

function Interpolations.linear_interpolation(times, traj::NamedTrajectory; kwargs...)
    return trajectory_interpolation(times, traj, :linear; kwargs...)
end

function Interpolations.constant_interpolation(times, traj::NamedTrajectory; kwargs...)
    return trajectory_interpolation(times, traj, :constant; kwargs...)
end

# *************************************************************************** #

@testitem "linear interpolation basic functionality" begin
    using Interpolations: linear_interpolation

    # Create a simple test trajectory
    T = 10
    timesteps = fill(1.0, T)
    times = collect(0.0:1.0:T-1)
    data = stack([sin.(times), cos.(times), timesteps], dims=1)
    comps = (x = 1:1, y = 2:2, Δt = 3:3)
    traj = NamedTrajectory(vec(data), comps, T; timestep=:Δt)

    new_times = collect(range(times[1], times[end], length=2T))
    new_traj = linear_interpolation(new_times, traj)

    @test size(new_traj.data, 2) == 2T

    # Check that the timestep component is correct
    @test new_traj[:Δt][1:end-1] ≈ diff(new_times)

    # Check that interpolating at original times gives back original data
    orig_traj = linear_interpolation(times, traj)
    @test orig_traj[:x] ≈ traj[:x]
    @test orig_traj[:y] ≈ traj[:y]
end

@testitem "linear interpolation with constant data" begin
    using Interpolations: linear_interpolation

    # Create a constant test trajectory
    T = 10
    timesteps = fill(1.0, T)
    times = collect(0.0:1.0:T-1)
    data = stack([ones(length(times)), zeros(length(times)), timesteps], dims=1)
    comps = (x = 1:1, y = 2:2, Δt = 3:3)
    traj = NamedTrajectory(vec(data), comps, T; timestep=:Δt)

    # Check that constant data is interpolated correctly
    new_times = collect(range(times[1], times[end], length=2T))
    new_traj = linear_interpolation(new_times, traj)

    @test size(new_traj.data, 2) == 2T

    # Check that the timestep component is correct
    @test new_traj[:Δt][1:end-1] ≈ diff(new_times)

    # Check that new data matches the constant values
    @test all(new_traj[:x] .== 1.0)
    @test all(new_traj[:y] .== 0.0)
end

end