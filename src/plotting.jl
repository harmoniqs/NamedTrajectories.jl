module Plotting

export trajectoryplot
export trajectoryplot!
export plot_trajectory
export plot_name
export plot_name!

"""
    trajectoryplot(traj::NamedTrajectory, components::Vector{Symbol}; kwargs...)
    trajectoryplot!(ax, traj::NamedTrajectory, components::Vector{Symbol}; kwargs...)

Plot components of a `NamedTrajectory` using Makie.
"""
function trajectoryplot end
function trajectoryplot! end

"""
    plot_trajectory(traj::NamedTrajectory, components::Vector{Symbol}; kwargs...)

Plot a `NamedTrajectory` and return a `Figure`.
"""
function plot_trajectory end 

"""
    plot_name(traj::NamedTrajectory, name::Symbol; kwargs...)
    plot_name!(ax, traj::NamedTrajectory, name::Symbol; kwargs...)

Plot a single component of a `NamedTrajectory` using Makie.
"""
function plot_name end
function plot_name! end

end