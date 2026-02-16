module PlottingExt

using NamedTrajectories
import NamedTrajectories: trajectoryplot, trajectoryplot!, plot_trajectory, plot_name, plot_name!

# recommended to use Makie for ext
using Makie
import Makie: plot
using LaTeXStrings
using TestItems

const AbstractTransform = Union{<:Function, AbstractVector{<:Function}}

const TrajType = Union{NamedTrajectory, Observable{<:NamedTrajectory}}

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

    label = nothing
    merge = false
    transform = nothing
    
    # added to avoid errors from old usage
    free_time = false
end

function Makie.plot!(p::TrajectoryPlot)
    traj = p.traj
    # Handle single symbol or vector of symbols
    comps_val = to_value(p.components)
    components_list = comps_val isa Symbol ? [comps_val] : comps_val

    # Time axis (shared across all)
    times = lift(traj, p.indices) do t, idxs
        Δt = t[t.timestep]
        ts = [0.0; cumsum(vec(Δt)[1:end-1])]
        return isnothing(idxs) ? ts : ts[idxs]
    end

    # Loop over each requested component
    for comp in components_list
        # Initial values to get dimensions (assume fixed during observable updates)
        init_traj = to_value(traj)
        init_data = init_traj[comp]
        
        # Apply initial transform to get K
        trans = to_value(p.transform)
        if isnothing(trans)
            K = init_data isa AbstractVector ? 1 : size(init_data, 1)
        elseif trans isa Vector
            # Vector transform implies per-timestep transformation (zip with columns)
            # Determine K from the output of the transform on the first column
            val = trans[1](init_data[:, 1])
            K = length(val)
        else
            transformed = trans.(eachcol(init_data))
            if !isa(transformed[1], AbstractVector)
                K = length(transformed[1])
            else
                K = size(stack(transformed), 1)
            end
        end

        colors = Makie.resample_cmap(to_value(p.color), max(2, K))
        
        # Plot each dimension of the component
        for i in 1:K
            # Lift data for this specific dimension
            data_i = lift(traj, p.transform, p.indices) do t, tr, idxs
                d = t[comp]
                # Transform
                if !isnothing(tr)
                    if tr isa Vector
                        d = stack([T(col) for (T, col) in zip(tr, eachcol(d))])
                    else
                        d = stack(tr.(eachcol(d)))
                    end
                end
                
                val = if d isa AbstractVector
                    d
                else
                    d[i, :]
                end
                return isnothing(idxs) ? collect(val) : collect(val)[idxs]
            end

            # Generate label
            lbl = lift(p.label, p.merge) do base_label, m
                if base_label isa AbstractVector
                     # If label is a vector, assume it matches K dimensions
                     return i <= length(base_label) ? base_label[i] : latexstring(string(comp))
                end

                if isnothing(base_label)
                    b = string(comp)
                elseif base_label isa LaTeXString
                    if m || K == 1
                        return base_label
                    end
                    # Extract inner content from $...$ wrapping
                    s = String(base_label)
                    b = startswith(s, "\$") && endswith(s, "\$") ? s[2:end-1] : s
                else
                    b = base_label
                end

                if m || K == 1
                    return latexstring(b)
                else
                    return latexstring(b, "_{", i, "}")
                end
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

# -------------------------------------------------------------- #
# Aliases for plot_name and plot_name!
# -------------------------------------------------------------- #

function NamedTrajectories.trajectoryplot(traj::TrajType, args...; kwargs...)
    plot_trajectory(traj, args...; kwargs...)
end

function NamedTrajectories.plot_name!(ax::Axis, traj::TrajType, name::Symbol; kwargs...)
    trajectoryplot!(ax, traj, name; kwargs...)
end

function NamedTrajectories.plot_name!(ax::Axis, traj::TrajType, input::Symbol, output::AbstractString, transform; kwargs...)
    trajectoryplot!(ax, traj, input, output, transform; kwargs...)
end

function NamedTrajectories.plot_name!(ax::Axis, traj::TrajType, input::Symbol, transform; kwargs...)
    trajectoryplot!(ax, traj, input, transform; kwargs...)
end

function NamedTrajectories.plot_name(traj::TrajType, args...; kwargs...)
    trajectoryplot(traj, args...; kwargs...)
end

# -------------------------------------------------------------- #
# Trajectory Plot methods
# -------------------------------------------------------------- #

function NamedTrajectories.trajectoryplot!(ax::Axis, traj::TrajType, name::Symbol; kwargs...)
    trajectoryplot!(ax, traj, [name]; kwargs...)
end

function NamedTrajectories.trajectoryplot!(ax::Axis, traj::TrajType, input::Symbol, transform; kwargs...)
    trajectoryplot!(ax, traj, [input]; transform=transform, kwargs...)
end

function NamedTrajectories.trajectoryplot!(ax::Axis, traj::TrajType, input::Symbol, output::AbstractString, transform; kwargs...)
    trajectoryplot!(ax, traj, [input]; label=output, transform=transform, kwargs...)
end

function NamedTrajectories.trajectoryplot!(ax::Axis, traj::TrajType, components::AbstractVector{Symbol}, transform; kwargs...)
    trajectoryplot!(ax, traj, components; transform=transform, kwargs...)
end

function NamedTrajectories.trajectoryplot!(ax::Axis, traj::TrajType, components::AbstractVector{Symbol}, output::AbstractString, transform; kwargs...)
    trajectoryplot!(ax, traj, components; label=output, transform=transform, kwargs...)
end

# -------------------------------------------------------------- #
# Figure Plotting
# -------------------------------------------------------------- #

function Makie.plot(
    traj::TrajType,
    names::Union{Symbol, AbstractVector{Symbol}, Tuple{Vararg{Symbol}}}=filter(x -> x != to_value(traj).timestep, to_value(traj).names);

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
    t = to_value(traj)

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
        if !use_autolimits && name in keys(t.bounds)
            ymin = minimum(t.bounds[name][1])
            ymax = maximum(t.bounds[name][2])
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
        plt = trajectoryplot!(ax, traj, name; merge=merge, kwargs...)
        # Legend construction
        child_plots = filter(p -> haskey(p, :label) && !isnothing(to_value(p.label)), plt.plots)
        labels = [to_value(p.label) for p in child_plots]
        # Deduplicate legend entries when labels are merged
        if merge
            seen = Set{String}()
            keep = Int[]
            for (j, lbl) in enumerate(labels)
                s = string(lbl)
                if s ∉ seen
                    push!(seen, s)
                    push!(keep, j)
                end
            end
            child_plots = child_plots[keep]
            labels = labels[keep]
        end
        Legend(fig[i, 2], child_plots, labels, tellheight=false)
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
            plt = trajectoryplot!(ax, traj, input, output, transform; merge=merge, kwargs...)
        else
            plt = trajectoryplot!(ax, traj, input, transform; merge=merge, kwargs...)
        end
        # Legend construction
        child_plots = filter(p -> haskey(p, :label) && !isnothing(to_value(p.label)), plt.plots)
        labels = [to_value(p.label) for p in child_plots]
        # Deduplicate legend entries when labels are merged
        if merge
            seen = Set{String}()
            keep = Int[]
            for (j, lbl) in enumerate(labels)
                s = string(lbl)
                if s ∉ seen
                    push!(seen, s)
                    push!(keep, j)
                end
            end
            child_plots = child_plots[keep]
            labels = labels[keep]
        end
        Legend(fig[offset + i, 2], child_plots, labels, tellheight=false)
    end

    for i in 1:length(transformations)-1
        rowgap!(fig.layout, offset + i, 0.0)
    end

    fig
end

function NamedTrajectories.plot_trajectory(traj::TrajType, args...; kwargs...)
    return Makie.plot(traj, args...; kwargs...)
end


@testitem "check point based" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)

    f, ax, plt = stairs(traj, 1)
    @test f isa Figure
end

@testitem "check default plot is series" begin
    using CairoMakie
    f = plot(Observable(rand(NamedTrajectory, 10, state_dim=3)), [:x])
    @test f isa Figure
end

@testitem "convert_arguments plot with transform" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)

    f = Figure()
    trajectoryplot(f[1,1], traj, :x)

    ax = Axis(f[2, 1])
    labels = ["T(x) $i" for i in 1:size(traj.x, 1)]
    p = trajectoryplot!(
        ax, traj, :x, x -> x .^ 2,
        label=labels, color=:seaborn_colorblind, marker=:circle,
    )
    # Check labels on the individual lines
    # p.plots contains Lines objects.
    # Verify we have correct number of plots and labels match
    @test length(p.plots) >= size(traj.x, 1) # Might include scatter plots
    # Filter for Lines plots
    line_plots = filter(x -> x isa Lines, p.plots)
    @test length(line_plots) == size(traj.x, 1)
    for (i, lp) in enumerate(line_plots)
        @test lp.label[] == labels[i]
    end
    @test f isa Figure
end

@testitem "basic Plot_Name recipe" begin
    using CairoMakie

    traj = rand(NamedTrajectory, 10, state_dim=3)

    f = Figure()
    ax = Axis(f[1,1])
    p = plot_name!(ax, traj, :x)

    @test p isa Plot

    # Test Series attributes (now on the recipe object or distributed)
    # The recipe object p has the attributes passed to it.
    for attr in [:color, :linestyle, :linewidth, :marker, :markersize]
        @test haskey(p.attributes, attr)
    end

    # Test labels
    # The recipe creates multiple lines.
    # We check if the lines have the correct labels.
    line_plots = filter(x -> x isa Lines, p.plots)
    @test length(line_plots) == size(traj.x, 1)

    expected_labels = ["\$x_{$i}\$" for i in 1:size(traj.x, 1)]
    for (i, lp) in enumerate(line_plots)
        @test lp.label[] == expected_labels[i]
    end

    @test f isa Figure
end

@testitem "Plot_Name with one dimension" begin
    using CairoMakie
    f = plot_name(rand(NamedTrajectory, 10, state_dim=1), :x)
    @test f isa Figure
end

@testitem "transform with vector of functions" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=1)
    # Use trajectoryplot! to get the plot object for inspection
    f = Figure()
    ax = Axis(f[1,1])
    
    p = trajectoryplot!(ax, traj, :x, [x -> [t - 1] for t in 1:10])

    # Extract data from the plot
    line_plot = p.plots[1]
    
    # Makie converts (x, y) to Points internally for Lines
    points = to_value(line_plot[1])
    plotted_data = [p[2] for p in points]
    
    expected_y = [Float64(t) for t in 0:9]
    
    @test plotted_data ≈ expected_y
    @test f isa Figure
end

@testitem "Plot_Name with many colors" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=100)

    f = Figure()
    ax = Axis(f[1,1])
    p = plot_name!(ax, traj, :x)
    # extract data: 
    # Should have 100 Lines plots
    line_plots = filter(x -> x isa Lines, p.plots)
    @test length(line_plots) == 100
    @test f isa Figure
end

@testitem "Plot_Name with indices" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)

    f = Figure()
    ax = Axis(f[1,1])
    indices = [1, 3, 5]
    # indices is passed as kwarg
    p = plot_name!(ax, traj, :x, indices=indices)

    # extract data:
    # There should be 3 lines (state_dim=3)
    line_plots = filter(x -> x isa Lines, p.plots)
    
    # Check the first dimension
    points = to_value(line_plots[1][1])
    plotted_times = [p[1] for p in points]
    plotted_data = [p[2] for p in points]
    
    times = get_times(traj)
    expected_times = times[indices]
    expected_data = traj.x[1, indices]
    
    @test plotted_times ≈ expected_times
    @test plotted_data ≈ expected_data
end

@testitem "trajectoryplot alias" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)
    f = trajectoryplot(traj, :x)
    @test f isa Figure
end

@testitem "Plot_Name transform and merge" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)

    f = Figure()
    ax = Axis(f[1,1])
    p = plot_name!(ax, traj, :x, "y", x -> x .^ 2, linewidth=3, marker=:circle, merge=true)

    # With merge=true, labels should be just L"$y$"
    line_plots = filter(x -> x isa Lines, p.plots)
    for lp in line_plots
        @test lp.label[] == "\$y\$"
    end

    ax = Axis(f[2,1])
    p = plot_name!(ax, traj, :x, "y", x -> x .^ 2, linewidth=3, marker=:circle)

    # With merge=false (default), labels should be L"$y_{1}$", L"$y_{2}$", etc.
    line_plots = filter(x -> x isa Lines, p.plots)
    for (i, lp) in enumerate(line_plots)
        @test lp.label[] == "\$y_{$i}\$"
    end
end

@testitem "LaTeXString label passthrough without double-wrapping" begin
    using CairoMakie
    using LaTeXStrings
    traj = rand(NamedTrajectory, 10, state_dim=3)

    f = Figure()

    # Merged: LaTeXString label should be returned as-is
    ax = Axis(f[1, 1])
    p = plot_name!(ax, traj, :x, L"$\alpha$", x -> x .^ 2, merge=true)
    line_plots = filter(x -> x isa Lines, p.plots)
    for lp in line_plots
        @test lp.label[] == L"$\alpha$"
    end

    # Not merged: should append subscript without double-wrapping $
    ax = Axis(f[2, 1])
    p = plot_name!(ax, traj, :x, L"$\alpha$", x -> x .^ 2)
    line_plots = filter(x -> x isa Lines, p.plots)
    for (i, lp) in enumerate(line_plots)
        @test lp.label[] == latexstring("\\alpha_{$i}")
    end
end

@testitem "traj plot merge" begin
    using CairoMakie
    state_dim = 3
    control_dim = 2
    traj = rand(NamedTrajectory, 10, state_dim=state_dim, control_dim=control_dim)

    # false, false (default)
    f = plot(traj)
    # Check axis 1 (states)
    ax1 = f.content[1] # Axis

    # Helper to find Lines plots recursively
    function find_lines(scene)
        lines = []
        for plot in scene.plots
            if plot isa Lines
                push!(lines, plot)
            elseif hasattr(plot, :plots) && !isempty(plot.plots)
                append!(lines, find_lines(plot))
            end
        end
        return lines
    end

    # Use Makie's collect_atomic_plots if available or manual traversal
    # ax1.scene.plots contains the recipe plot.
    # The recipe plot contains Lines.

    # Manual traversal for safety
    all_plots = []
    for p in ax1.scene.plots
        append!(all_plots, p.plots)
    end
    lines1 = filter(p -> p isa Lines, all_plots)

    @test length(lines1) == state_dim
    # Check labels - should be L"$x_{1}$", L"$x_{2}$", L"$x_{3}$"
    @test all(l -> l.label[] != "\$x\$", lines1)

    # true, true
    f = plot(traj, merge_labels=true)
    ax1 = f.content[1]

    all_plots = []
    for p in ax1.scene.plots
        append!(all_plots, p.plots)
    end
    lines1 = filter(p -> p isa Lines, all_plots)

    @test length(lines1) == state_dim
    # Check labels - all should be L"$x$" (merged)
    @test all(l -> l.label[] == "\$x\$", lines1)
end

@testitem "merge_labels legend deduplication" begin
    using CairoMakie
    state_dim = 3
    control_dim = 2
    traj = rand(NamedTrajectory, 10, state_dim=state_dim, control_dim=control_dim)

    # merge_labels=true should produce one legend entry per component
    f = plot(traj, merge_labels=true)
    # Row 1: x with 3 dims → legend should have 1 entry
    legend1 = contents(f[1, 2])[1]
    _, entries1 = legend1.entrygroups[][1]
    @test length(entries1) == 1

    # Row 2: u with 2 dims → legend should have 1 entry
    legend2 = contents(f[2, 2])[1]
    _, entries2 = legend2.entrygroups[][1]
    @test length(entries2) == 1

    # Without merge: legend entries should match dimensionality
    f2 = plot(traj)
    legend1_nomerge = contents(f2[1, 2])[1]
    _, entries1_nomerge = legend1_nomerge.entrygroups[][1]
    @test length(entries1_nomerge) == state_dim

    legend2_nomerge = contents(f2[2, 2])[1]
    _, entries2_nomerge = legend2_nomerge.entrygroups[][1]
    @test length(entries2_nomerge) == control_dim
end

@testitem "merge_labels mixed per-component" begin
    using CairoMakie
    state_dim = 3
    control_dim = 2
    traj = rand(NamedTrajectory, 10, state_dim=state_dim, control_dim=control_dim)

    # merge_labels=[false, true]: x unmerged, u merged
    f = plot(traj, [:x, :u], merge_labels=[false, true])

    # Row 1: x with merge=false → state_dim legend entries
    legend1 = contents(f[1, 2])[1]
    _, entries1 = legend1.entrygroups[][1]
    @test length(entries1) == state_dim

    # Row 2: u with merge=true → 1 legend entry
    legend2 = contents(f[2, 2])[1]
    _, entries2 = legend2.entrygroups[][1]
    @test length(entries2) == 1
end

@testitem "merge_labels with transformations" begin
    using CairoMakie
    state_dim = 3
    control_dim = 2
    traj = rand(NamedTrajectory, 10, state_dim=state_dim, control_dim=control_dim)

    # Test merge_transformation_labels deduplicates transformation legends
    transformations = [(:x => x -> abs.(x)), (:u => u -> u .^ 2)]
    f = plot(
        traj, Symbol[];
        transformations=transformations,
        merge_transformation_labels=true
    )
    # Row 1: transform of x (3 dims) with merge → 1 legend entry
    legend1 = contents(f[1, 2])[1]
    _, entries1 = legend1.entrygroups[][1]
    @test length(entries1) == 1

    # Row 2: transform of u (2 dims) with merge → 1 legend entry
    legend2 = contents(f[2, 2])[1]
    _, entries2 = legend2.entrygroups[][1]
    @test length(entries2) == 1

    # Mixed: merge_labels=[false, true] on components + merge_transformation_labels=[true, false] on transforms
    f2 = plot(
        traj, [:x, :u];
        merge_labels=[false, true],
        transformations=transformations,
        merge_transformation_labels=[true, false]
    )

    # Row 1 (x, unmerged): state_dim entries
    _, entries_x = contents(f2[1, 2])[1].entrygroups[][1]
    @test length(entries_x) == state_dim

    # Row 2 (u, merged): 1 entry
    _, entries_u = contents(f2[2, 2])[1].entrygroups[][1]
    @test length(entries_u) == 1

    # Row 3 (transform x, merged): 1 entry
    _, entries_tx = contents(f2[3, 2])[1].entrygroups[][1]
    @test length(entries_tx) == 1

    # Row 4 (transform u, unmerged): control_dim entries
    _, entries_tu = contents(f2[4, 2])[1].entrygroups[][1]
    @test length(entries_tu) == control_dim
end

@testitem "traj plot with transformations" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)

    # multiple transformations
    transformations = [(:x => x -> [x[1] * 30]), (:u => u -> u .^2)]

    f = plot(
        traj, [:u],
        transformations=transformations,
        transformation_labels=["Label(x)", "Label(u)"], 
        merge_transformation_labels=[false, true]
    )
    # check for axes
    axes = filter(c -> c isa Axis, f.content)
    @test length(axes) == 3

    # test repeat label
    push!(transformations, (:x => x -> [x[2] ^6]))
    f = plot(
        traj, 
        transformations=transformations,
    )
    axes = filter(c -> c isa Axis, f.content)
    @test length(axes) == 5
    @test f isa Figure
end

@testitem "transformation titles" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)

    # multiple transformations
    transformations = [(:x => x -> [x[1] * 30]), (:u => u -> u .^2)]
    transformation_titles= ["Title 1", "Title 2"]

    f = plot(
        traj, Symbol[],
        transformations=transformations,
        transformation_titles=transformation_titles
    )
    axes = filter(c -> c isa Axis, f.content)
    @test axes[1].subtitle[] == transformation_titles[1]
    @test axes[2].subtitle[] == transformation_titles[2]

end

@testitem "traj plot with series kwargs" begin
    using CairoMakie
    # test passing in series kwargs
    f = plot(
        rand(NamedTrajectory, 10, state_dim=3), 
        [:x, :u],
        linewidth=5,
        color=:glasbey_bw_minc_20_n256,
        marker = :x,
        markersize = 10
    )
    # check attributes of first axis
    attributes = f.content[1].scene.plots[1].attributes
    @test attributes.linewidth[] == 5
    @test attributes.color[] == :glasbey_bw_minc_20_n256
    @test attributes.marker[] == :x
    @test attributes.markersize[] == 10
    @test f isa Figure
end

@testitem "create figure with a theme" begin
    using CairoMakie
    f = with_theme(theme_dark()) do
        plot(rand(NamedTrajectory, 10, state_dim=3))
    end
    @test f isa Figure
end

@testitem "default plots" begin
    using CairoMakie
    # test passing in series kwargs
    f = plot(rand(NamedTrajectory, 10, state_dim=3), free_time=true)
    @test f isa Figure
    f = plot(rand(NamedTrajectory, 10, state_dim=3), free_time=false)
    @test f isa Figure
end

@testitem "multiple components in trajectoryplot!" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3, control_dim=2)
    f = Figure()
    ax = Axis(f[1, 1])
    p = trajectoryplot!(ax, traj, [:x, :u])

    # Check for Lines from both components
    # :x has 3 dims, :u has 2 dims -> total 5 line plots
    line_plots = filter(x -> x isa Lines, p.plots)
    @test length(line_plots) == 5

    # Verify labels contain both x and u (LaTeXStrings contain the symbol names)
    labels = [string(lp.label[]) for lp in line_plots]
    @test any(contains("x"), labels)
    @test any(contains("u"), labels)
end

@testitem "markersize and scatter plots" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=2)
    f = Figure()
    ax = Axis(f[1, 1])
    p = trajectoryplot!(ax, traj, :x, markersize=5, marker=:circle)
    
    # Should have 2 Lines and 2 Scatters
    line_plots = filter(x -> x isa Lines, p.plots)
    scatter_plots = filter(x -> x isa Scatter, p.plots)
    
    @test length(line_plots) == 2
    @test length(scatter_plots) == 2
    
    # Check markersize attribute
    @test all(scatter_plots[1].markersize[] .≈ 5)
end

@testitem "custom colormap and unique colors" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)
    f = Figure()
    ax = Axis(f[1, 1])
    cmap = :viridis
    p = trajectoryplot!(ax, traj, :x, color=cmap)
    
    line_plots = filter(x -> x isa Lines, p.plots)
    colors = [lp.color[] for lp in line_plots]
    
    # Colors should be unique if sampled from colormap
    @test length(unique(colors)) == 3
end

@testitem "labels on transformed traj on one vector use correct dim of vector for color/label sampling" begin
    using CairoMakie
    
    traj = rand(NamedTrajectory, 10, state_dim=3, control_dim=2)
    plt = plot(traj, transformations = [(:x => x -> [x[1]])])
    @test plt isa Figure    
    # transformed plot and legend are: 3rd plot (row) and legend is in 2nd column.
    # entrygroups[] is a vector (usuall of length 1) of tuples (legend title, legend entries)
    _, transformed_legend_entries = contents(plt[3, 2])[1].entrygroups[][1]
    @test length(transformed_legend_entries) == 1
    
    plt = plot(traj, transformations = [(:x => x -> x[1:1])])
    @test plt isa Figure
    _, transformed_legend_entries = contents(plt[3, 2])[1].entrygroups[][1]
    @test length(transformed_legend_entries) == 1
    
    # special scalar case
    plt = plot(traj, transformations = [(:x => x -> x[1])])
    @test plt isa Figure
    _, transformed_legend_entries = contents(plt[3, 2])[1].entrygroups[][1]
    @test length(transformed_legend_entries) == 1
end

end