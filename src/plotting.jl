module Plotting

export plot_name
export plot_name!

"""
    plot_name(traj::NamedTrajectory, name::Symbol; kwargs...)

Plot a single component of a `NamedTrajectory` using Makie.

The default plot type is `Series`. Series attributes can be passed as keyword arguments.

"""
function plot_name end

function plot_name! end

end