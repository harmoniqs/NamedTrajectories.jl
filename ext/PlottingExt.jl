module PlottingExt

using NamedTrajectories
# We don't use the recipe anymore, but we implement the function
import NamedTrajectories: plot_name, plot_name!

using Makie
# using ComputePipeline # Removed to avoid confusion

const AbstractTransform = Union{<:Function, AbstractVector{<:Function}}

# -------------------------------------------------------------- #
# Helper functions
# -------------------------------------------------------------- #

function apply_transform(traj, name, transform)
    if !isnothing(transform)
        try
            if transform isa Vector
                return stack([T(col) for (T, col) in zip(transform, eachcol(traj[name]))])
            else
                return stack(transform.(eachcol(traj[name])))
            end
        catch
            throw(ArgumentError("Transformation of $(name) failed."))
        end
    else
        return traj[name]
    end
end

# -------------------------------------------------------------- #
# Plot trajectories by name (Direct implementation bypassing recipe)
# -------------------------------------------------------------- #

"""
    plot_name!(ax, traj, name; kwargs...)

Plots a component of the trajectory on the given axis.
"""
function NamedTrajectories.plot_name!(
    ax::Axis,
    traj::NamedTrajectory,
    name::Symbol,
    output_name::AbstractString = string(name),
    transform::Union{Nothing, AbstractTransform} = nothing;
    merge::Bool = false,
    color = :glasbey_bw_n256,
    linestyle = nothing,
    linewidth = 1.5,
    marker = nothing,
    markersize = nothing,
    indices::Union{Nothing, AbstractVector{Int}} = nothing,
    kwargs...
)
    # Get data and times
    data = apply_transform(traj, name, transform)
    
    # If 1D, convert to 2D
    if data isa AbstractVector
        data = reshape(data, 1, :)
    end

    all_times = get_times(traj)
    
    # Handle indices
    if !isnothing(indices)
        times = all_times[indices]
        data = data[:, indices]
    else
        times = all_times
    end

    # Handle labels
    if merge
        labels = fill(output_name, size(data, 1))
    else
        labels = ["$(output_name) $i" for i in 1:size(data, 1)]
    end

    # Resample colors
    if color isa Symbol || color isa Vector{<:Makie.Colorant} || color isa String
         c = Makie.resample_cmap(color, max(2, length(labels)))
    else
         c = color 
    end

    series!(
        ax, times, data;
        labels = labels,
        color = c,
        linestyle = linestyle,
        linewidth = linewidth,
        marker = marker,
        markersize = isnothing(markersize) ? 5.0 : markersize,
        kwargs...
    )
end

# Overload for transformed plot with transform object (explicit)
function NamedTrajectories.plot_name!(
    ax::Axis,
    traj::NamedTrajectory,
    name::Symbol,
    transform::AbstractTransform;
    kwargs...
)
    plot_name!(ax, traj, name, "T($(name))", transform; kwargs...)
end

# -------------------------------------------------------------- #
# Plot trajectories as figure
# -------------------------------------------------------------- #

"""
    Makie.plot(
        traj::NamedTrajectory,
        names::Union{AbstractVector{Symbol}, Tuple{Vararg{Symbol}}}=traj.names;
        kwargs...
    )

Plot a `NamedTrajectory` using Makie.

"""
function Makie.plot(
    traj::NamedTrajectory,
    names::Union{AbstractVector{Symbol}, Tuple{Vararg{Symbol}}}=filter(x -> x != traj.timestep, traj.names);

    # ---------------------------------------------------------------------------
    # component specification keyword arguments
    # ---------------------------------------------------------------------------
    
    # whether or not to include unique labels for components
    merge_labels::Union{Bool, AbstractVector{Bool}} = false,

    # autolimits will use trajectory data and ignore trajectory bounds
    use_autolimits::Bool = false,

    # ---------------------------------------------------------------------------
    # transformation keyword arguments
    # ---------------------------------------------------------------------------

    # transformations, e.g., [(:x => x -> [x[1]; abs(x[2])]), ...]
    transformations::AbstractVector{<:Pair{Symbol, <:AbstractTransform}} = Pair{Symbol, Function}[],

    # labels for transformed components
    transformation_labels::AbstractVector{<:AbstractString} = fill("", length(transformations)),

    # titles for transformations
    transformation_titles::AbstractVector{<:AbstractString} = fill("", length(transformations)),

    # whether or not to include unique labels for transformed components
    merge_transformation_labels::Union{Bool, AbstractVector{Bool}} = false,
    
    # ---------------------------------------------------------------------------
    # figure and axis keyword arguments
    # ---------------------------------------------------------------------------
    
    fig_size::Tuple{Int, Int} = (800, 600),
    titlesize::Int=18,
    subtitlesize::Int=16,
    xlabelsize::Int=16,

    # ---------------------------------------------------------------------------
    # plot keyword arguments (for all names)
    # ---------------------------------------------------------------------------
    kwargs...
)

    # parse arguments
    if names isa Symbol
        names = [names]
    end

    if merge_labels isa Bool
        merge_labels = fill(merge_labels, length(names))
    end

    if merge_transformation_labels isa Bool
        merge_transformation_labels = fill(merge_transformation_labels, length(transformations))
    end

    @assert length(merge_transformation_labels) == length(transformation_labels) == length(transformations)

    # create figure
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
        merge = merge_labels[i]
        
        # Call the direct plot_name! function
        plot_name!(ax, traj, name; merge=merge, kwargs...)
        
        # Add legend
        Legend(fig[i, 2], ax, merge=merge)
    end

    for i in 1:length(names) - 1
        rowgap!(fig.layout, i, Relative(0.015))
    end


    # Transformations
    # ---------------
    offset = length(names)

    for (i, (input, transform)) in enumerate(transformations)
        ax = Axis(
            fig[offset + i, 1],
            title = i == 1 || !isempty(transformation_titles[i]) ? "Transformations" : "",
            titlecolor = i == 1 ? theme(fig.scene, :textcolor) : (:black, 0.0),
            titlealign = :left,
            titlesize = titlesize,
            subtitle = transformation_titles[i],
            subtitlesize = subtitlesize,
            xticklabelsvisible = i == length(transformations),
            xtickalign = 1,
            xlabel = i == length(transformations) ? "time" : "",
            xlabelsize = xlabelsize,
        )
        
        output = transformation_labels[i]
        merge = merge_transformation_labels[i]
        
        if !isempty(output)
            plot_name!(ax, traj, input, output, transform; merge=merge, kwargs...)
        else
            plot_name!(ax, traj, input, transform; merge=merge, kwargs...)
        end
        Legend(fig[offset + i, 2], ax, merge=merge)
    end

    for i in 1:length(transformations)-1
        rowgap!(fig.layout, offset + i, 0.0)
    end

    fig
end

function Makie.plot(
    theme::Makie.Theme,
    args...;
    kwargs...
)
    with_theme(theme) do
        plot(args...; kwargs...)
    end
end

# =========================================================================== #
# Test Items (Commented out to avoid extension loading issues)
# =========================================================================== #

# @testitem "check point based" begin
#     using CairoMakie
#     traj = rand(NamedTrajectory, 10, state_dim=3)
#     f, ax, plt = stairs(traj, 1)
#     @test f isa Figure
# end

# @testitem "check default plot is series" begin
#     using CairoMakie
#     f = plot(rand(NamedTrajectory, 10, state_dim=3))
#     @test f isa Figure
# end
end