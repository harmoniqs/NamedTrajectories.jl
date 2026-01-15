module Plotting

export trajectoryplot
export trajectoryplot!
export  plot_trajectory

"""
    plot_name(traj::NamedTrajectory, name::Symbol; kwargs...)

Plot a single component of a `NamedTrajectory` using Makie.

The default plot type is `Series`. Series attributes can be passed as keyword arguments.

"""
function trajectoryplot end

function trajectoryplot! end

function plot_trajectory end 


end