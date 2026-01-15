module PlottingExt

using NamedTrajectories
import NamedTrajectories: trajectoryplot, trajectoryplot!, plot_trajectory

# recommended to use Makie for ext
using Makie
using TestItems

const AbstractTransform = Union{<:Function, AbstractVector{<:Function}}

# -------------------------------------------------------------- #
# Plot trajectories with PointBased conversion
# -------------------------------------------------------------- #

function Makie.convert_arguments(
    P::Makie.PointBased,
    traj::NamedTrajectory,
    comp::Int;
    indices::AbstractVector{Int}=1:traj.N
)
    times = get_times(traj)[indices]
    positions = map(zip(indices, times)) do (i, t)
        (t, traj.data[comp, i])
    end
    return Makie.convert_arguments(P, positions)
end

# -------------------------------------------------------------- #
# Plot trajectories by name using Series or Plot
# -------------------------------------------------------------- #

function Makie.convert_arguments(
    P::Type{<:Series}, 
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
    return Makie.convert_arguments(P, times, transform_data[:, indices])
end

# Allow transform to be passed to the plotting function
Makie.used_attributes(::Type{<:Series}, ::NamedTrajectory, ::Symbol) = (:transform, :indices)

# -------------------------------------------------------------- #
# Plot trajectories by name with recipe
# -------------------------------------------------------------- #

@recipe TrajectoryPlot (traj, components) begin
    color = :glasbey_bw_n256

    linestyle = @inherit linestyle nothing
    linewidth = @inherit linewidth 2.0
    marker = @inherit marker :circle

    indices = nothing

    markersize = 0.0

    Makie.mixin_generic_plot_attributes()...
    label = nothing
end

function Makie.plot!(p::TrajectoryPlot)
    traj = p.traj
    # Handle single symbol or vector of symbols
    comps_val = to_value(p.components)
    components_list = comps_val isa Symbol ? [comps_val] : comps_val

    # HERE
    line_count = lift(traj) do t
        count = 0
        for comp in components_list
            d = t[comp]
            K = d isa AbstractVector ? 1 : size(d, 1)
            count += K
        end
        return count
    end

    # Time axis (shared across all)
    times = lift(traj) do t
        Δt = t[:Δt]
        ts = [0.0; cumsum(vec(Δt)[1:end-1])]
        return ts
    end

    # Loop over each requested component
    for comp in components_list
        # Check dimensions for this component
        init_traj = to_value(traj)
        init_data = init_traj[comp]
        K = init_data isa AbstractVector ? 1 : size(init_data, 1)

        colors = Makie.resample_cmap(to_value(p.color), max(2, K))
        
        # Plot each dimension of the component
        for i in 1:K
            # Check indices filtering
            inds = to_value(p.indices)
            if !isnothing(inds) && !(i in inds)
                continue
            end

            # Lift data for this specific dimension
            data_i = lift(traj) do t
                d = t[comp]
                val = if d isa AbstractVector
                    d
                else
                    d[i, :]
                end
                return collect(val)
            end

            # Generate label
            lbl = if K == 1
                "$comp"
            else
                "$comp$i"
            end

            lines!(p, times, data_i, 
                label = lbl, 
                color = colors[i], 
                linewidth = p.linewidth,
                linestyle = p.linestyle
            )
            
            # Optional Scatter
            if to_value(p.markersize) > 0
                scatter!(p, times, data_i,
                    color = colors[i],
                    marker = p.marker,
                    markersize = p.markersize
                )
            end
        end
    end
end

function plot_trajectory(traj::Observable{<:NamedTrajectory}, components::Vector{Symbol}=[:x, :u, :v];
                         fig_size=(800, 1000), use_autolimits=false)
    fig = Figure(size=fig_size)
    # Get initial value for bounds checking
    init_traj = to_value(traj)
    for (i, name) in enumerate(components)
        # Determine limits
        limits = (nothing, nothing)
        if !use_autolimits && hasproperty(init_traj, :bounds) && name in keys(init_traj.bounds)
            ymin = minimum(init_traj.bounds[name][1])
            ymax = maximum(init_traj.bounds[name][2])
            ybounds = isfinite(ymin) && isfinite(ymax) ? (ymin, ymax) : nothing
            limits = (nothing, ybounds)
        else
            # Fallback defaults
            if name == :x
                limits = (nothing, (-1.5, 1.5))
            elseif name == :u || name == :v
                limits = (nothing, (-1.0, 1.0))
            end
        end
        ax = Axis(
            fig[i, 1],
            title = "$name",
            titlealign = :left,
            titlesize = 18,
            xticklabelsvisible = i == length(components),
            xtickalign=1,
            xlabel = i == length(components) ? "Time (s)" : "",
            xlabelsize = 16,
            limits = limits
        )
        # Plot the component using the recipe
        plt = trajectoryplot!(ax, traj, name)
        # Legend construction
        child_plots = filter(p -> haskey(p, :label) && !isnothing(to_value(p.label)), plt.plots)
        labels = [to_value(p.label) for p in child_plots]
        Legend(fig[i, 2], child_plots, labels, tellheight=false)
    end
    # Adjust layout
    colsize!(fig.layout, 1, Relative(0.85))
    return fig
end

end