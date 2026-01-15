module PlottingExt

using NamedTrajectories
import NamedTrajectories: plot_name, plot_name!

const AbstractTransform = Union{<:Function, AbstractVector{<:Function}}

using Makie
using ComputePipeline
using TestItems


function Makie.convert_arguments(
    traj_plot::Makie.PointBased,
    traj::NamedTrajectory,
    comp::Int;
    indices::AbstractVector{Int}=1:traj.N
)
    times = get_times(traj)[indices]
    positions = map(zip(indices, times)) do (i, t)
        (t, traj.data[comp, i])
    end
    return Makie.convert_arguments(traj_plot, positions)
end

# -------------------------------------------------------------- #
# Plot trajectories by name using Series or Plot
# -------------------------------------------------------------- #

function Makie.convert_arguments(
    traj_plot::Type{<:Series}, 
    traj::NamedTrajectory,
    name::Symbol;
    transform::Union{Nothing, AbstractTransform}=nothing,
    indices::AbstractVector{Int}=1:traj.N
)
    if !isnothing(transform)
        transform_data = try
            if transform isa Vector
                stack([T(col) for (T, col) in zip(transform, eachcol(traj[name]))])
            else
                stack(transform.(eachcol(traj[name])))
            end
        catch
            throw(ArgumentError("Transformation of $(name) failed."))
        end
    else
        transform_data = traj[name]
    end

    # If 1D, convert to 2D
    if transform_data isa AbstractVector
        transform_data = reshape(transform_data, 1, :)
    end

    times = get_times(traj)[indices]
    return Makie.convert_arguments(traj_plot, times, transform_data[:, indices])
end

# Allow transform to be passed to the plotting function
Makie.used_attributes(::Type{<:Series}, ::NamedTrajectory, ::Symbol) = (:transform, :indices)


@recipe Plot_Name (traj, input_name, output_name) begin
    color = :glasbey_bw_n256
    
    linestyle = @inherit linestyle nothing
    linewidth = @inherit linewidth
    marker = @inherit marker

    indices = nothing

    markersize = 0.0

    Makie.mixin_generic_plot_attributes()...
end

function Makie.plot!(traj_plot::Plot_Name)
    name = isnothing(traj_plot.input_name) ? :x : traj_plot.input_name
    label = isnothing(traj_plot.output_name) ? L"$(traj_plot.input_name)" : traj_plot.output_name
    traj = ComputePipeline.resolve!(traj_plot.traj)

    # println(typeof.([label, name, traj_plot.traj]))
    labels = fill(label, length(traj.components[ComputePipeline.resolve!(name)]))
    colors =  Makie.resample_cmap(ComputePipeline.resolve!(traj_plot.color), max(2, length(labels)))

    # Empty marker means no size
    markersize = isnothing(traj_plot.marker) ? nothing :  traj_plot.markersize

    # Empty indices means all indices
    indices = isnothing(traj_plot.indices) ? range(1, traj.N) : traj_plot.indices

    series!(
        traj_plot,
        traj_plot.traj,
        name;
        labels = labels,
        color = colors,
        linestyle = traj_plot.linestyle,
        linewidth = traj_plot.linewidth,
        marker = traj_plot.marker,
        markersize = markersize,
        indices = indices
    )

    return traj_plot # not needed!
end


# """
#     Makie.plot(
#         traj::NamedTrajectory,
#         names::Union{AbstractVector{Symbol}, Tuple{Vararg{Symbol}}}=traj.names;
#         kwargs...
#     )

# Plot a `NamedTrajectory` using Makie.

# """
# function Makie.plot(
#     traj::NamedTrajectory,
#     names::Union{AbstractVector{Symbol}, Tuple{Vararg{Symbol}}}=filter(x -> x != traj.timestep, traj.names);
#     kwargs...
# )

    fig_size = (800, 600)
    titlesize = 18
    subtitlesize = 16
    xlabelsize = 16
    use_autolimits = false

    fig = Figure(size=fig_size)

    # Default components
        # ------------------
    for (i, name) in enumerate(names)

        # Use bounds to set axis limits
        if !use_autolimits && name in keys(traj.bounds)
            ymin = minimum(traj.bounds[name][1])
            ymax = maximum(traj.bounds[name][2])
            ybounds = isfinite(ymin) && isfinite(ymax) ? (ymin, ymax) : nothing
            limits = (nothing, ybounds)
        else
            limits = (nothing, nothing)
        end

        ax = Axis(
            fig[i, 1],
            title = i == 1 ? "Named Trajectory" : "",
            titlealign = :left,
            titlesize = titlesize,
            xticklabelsvisible = i == length(names),
            xtickalign=1,
            xlabel = i == length(names) ? "time" : "",
            xlabelsize = xlabelsize,
            limits = limits
        )
        
        plot_name!(ax, traj, name; kwargs...)
        Legend(fig[i, 2], ax)
    end
    
    for i in 1:length(names) - 1
        rowgap!(fig.layout, i, Relative(0.015))
    end

#     return fig
# end

# =========================================================================== #
@testitem "check point based" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)

    f, ax, plt = stairs(traj, 1)
    @test f isa Figure
end

@testitem "check default plot is series" begin
    using CairoMakie
    f, ax, plt = plot(rand(NamedTrajectory, 10, state_dim=3), :x)
    @assert plt isa Plot
end

end