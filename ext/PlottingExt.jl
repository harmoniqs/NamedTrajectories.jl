module PlottingExt

using NamedTrajectories
import NamedTrajectories: namedplot, namedplot!

# Ideally, we'd only need MakieCore for recipes
# But, we need Series, Axis, Figure etc. 
# And it's recommended to use Makie for ext
using Makie

using TestItems

const AbstractTransform = Union{<:Function, AbstractVector{<:Function}}

# -------------------------------------------------------------- #
# Plot trajectories with PointBased conversion
# -------------------------------------------------------------- #

function Makie.convert_arguments(
    P::Makie.PointBased,
    traj::NamedTrajectory,
    comp::Int
)
    positions = map(enumerate(get_times(traj))) do (i, t)
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
    transform::Union{Nothing, AbstractTransform}=nothing
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

    return Makie.convert_arguments(P, get_times(traj), transform_data)
end

# Allow transform to be passed to the plotting function
Makie.used_attributes(::Type{<:Series}, ::NamedTrajectory, ::Symbol) = (:transform,)

# -------------------------------------------------------------- #
# Plot trajectories by name with recipe
# -------------------------------------------------------------- #

# docstring in plotting.jl
@recipe(NamedPlot, traj, input_name, output_name, transform) do scene
    # Add any desired series attributes here
    Attributes(
        color = :glasbey_bw_n256,
        linestyle = theme(scene, :linestyle),
        linewidth = theme(scene, :linewidth),
        marker = theme(scene, :marker),
        markersize = 0.0,
        # merge: if true, all components are plotted with the same label
        merge = false
    )
end

# Add the ability to recall plot labels for a legend (extract series subplots)
Makie.get_plots(P::NamedPlot) = Makie.get_plots(P.plots[1])

# Plot existing component
function Makie.plot!(
    P::NamedPlot{<:Tuple{<:NamedTrajectory, Symbol, <:AbstractString}};
    kwargs...
)
    lift(P[:traj], P[:input_name], P[:output_name]) do traj, name, label

        if P[:merge][]
            labels = fill(label, length(traj.components[name]))
        else
            labels = [convert(typeof(label), "$(label) $(i)") for i in eachindex(traj.components[name])]
        end

        # Resample a minimum of 2 colors
        colors =  Makie.resample_cmap(P[:color][], max(2, length(labels)))

        # Empty marker means no size
        markersize = isnothing(P[:marker][]) ? nothing :  P[:markersize]

        series!(
            P, traj, name;
            labels = labels,
            color = colors,
            linestyle = P[:linestyle],
            linewidth = P[:linewidth],
            marker = P[:marker],
            markersize = markersize,
            kwargs...
        )
        
    end
    return P
end

# Plot existing component (LaTeX label from name)
function Makie.plot!(
    P::NamedPlot{<:Tuple{<:NamedTrajectory, Symbol}};
    kwargs...
)
    plot!(P, P[:traj], P[:input_name], L"%$(P[:input_name][])"; kwargs...)
    return P
end

# Plot transformed component
function Makie.plot!(
    P::NamedPlot{<:Tuple{<:NamedTrajectory, Symbol, <:AbstractString, <:AbstractTransform}};
    kwargs...
)
    lift(P[:traj], P[:input_name], P[:output_name], P[:transform]) do traj, input, output, transform

        if P[:merge][]
            labels = fill(output, length(traj.components[input]))
        else
            labels = [convert(typeof(output), "$(output) $(i)") for i in eachindex(traj.components[input])]
        end

        # Resample a minimum of 2 colors
        colors =  Makie.resample_cmap(P[:color][], max(2, length(labels)))

        # Empty marker means no size
        markersize = isnothing(P[:marker][]) ? nothing :  P[:markersize]

        series!(
            P, traj, input;
            transform = transform,
            labels = labels,
            color = colors,
            linestyle = P[:linestyle],
            linewidth = P[:linewidth],
            marker = P[:marker],
            markersize = markersize,
            kwargs...
        )
        
    end
    return P
end

# Plot transformed component (output label from name)
function Makie.plot!(
    P::NamedPlot{<:Tuple{<:NamedTrajectory, Symbol, <:AbstractTransform}};
    kwargs...
)   
    # transform is arg 3
    plot!(P, P[:traj], P[:input_name], L"T(%$(P[:input_name][]))", P[3]; kwargs...)
    return P
end

# Allow plot to be called as alias for NamedPlot
Makie.plottype(::NamedTrajectory, ::Symbol) = NamedPlot
Makie.plottype(::NamedTrajectory, ::Symbol, ::AbstractString) = NamedPlot
Makie.plottype(::NamedTrajectory, ::Symbol, ::AbstractTransform) = NamedPlot
Makie.plottype(::NamedTrajectory, ::Symbol, ::AbstractString, ::AbstractTransform) = NamedPlot

# -------------------------------------------------------------- #
# Plot trajectories as figure
# -------------------------------------------------------------- #

# TODO:
# - Should we have a default theme?
# - Allow for transformations to use the entire knot point? No symbol?

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
    names::Union{AbstractVector{Symbol}, Tuple{Vararg{Symbol}}}=traj.names;

    # ---------------------------------------------------------------------------
    # component specification keyword arguments
    # ---------------------------------------------------------------------------

    # whether or not to plot the timestep componenent
    ignore_timestep::Bool=true,
    
    # whether or not to include unique labels for components
    merge_labels::Union{Bool, AbstractVector{Bool}} = false,

    # autolimits will use trajectory data and ignore trajectory bounds
    use_autolimits::Bool = false,

    # ---------------------------------------------------------------------------
    # transformation keyword arguments
    # ---------------------------------------------------------------------------

    # transformations
    transformations::AbstractVector{<:Pair{Symbol, <:Union{<:Function, <:AbstractVector{<:Function}}}} = Pair{Symbol, Function}[],

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

    if traj.timestep isa Symbol && ignore_timestep
        names = filter(x -> x != traj.timestep, names)
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
        namedplot!(ax, traj, name, merge=merge; kwargs...)
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
        namedplot!(ax, traj, input, output, transform, merge=merge; kwargs...)
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

@testitem "convert_arguments plot with legend and transform" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)

    f = Figure()
    plot(f[1,1], traj, :x)
    
    ax = Axis(f[2, 1])
    labels = ["T(x) $i" for i in 1:size(traj.x, 1)]
    p = plot!(
        ax, traj, :x, x -> x .^ 2, 
        labels=labels, color=:seaborn_colorblind, marker=:circle,
    )
    Legend(f[2,2], ax)
    @test p.attributes.labels[] == labels
    @test f isa Figure
end

@testitem "basic namedplot recipe" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)

    f = Figure()
    ax = Axis(f[1,1])
    p = namedplot!(ax, traj, :x)
    Legend(f[1,2], ax)

    @test p isa Plot{namedplot}
    
    # Test Series attributes
    for attr in [:color, :linestyle, :linewidth, :marker, :markersize]
        @test attr in keys(p.attributes)
    end

    # Test special attributes
    @test p.attributes.merge[] == false

    # Test labels (set on the Series plot) (LaTeX string defaults)
    labels = p.plots[1].plots[1].attributes.labels[]
    @test labels == [L"$x$ %$i" for i in 1:size(traj.x, 1)]

    @test f isa Figure
end

@testitem "named plot with LaTeXString label" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)
    label = L"\alpha"
    _, _, p = namedplot(traj, :x, label, merge=true)
    # Check that LaTeX string is preserved
    @test p.plots[1].attributes.labels[][1] == label
end

@testitem "namedplot with one dimension" begin
    using CairoMakie
    f, ax, plt = namedplot(rand(NamedTrajectory, 10, state_dim=1), :x)
    @test f isa Figure
end

@testitem "transform with vector of functions" begin
    using CairoMakie
    f, ax, plt = plot(rand(NamedTrajectory, 10), :x, [x -> [t - 1] for t in 1:10])

    # check internals 
    expected = [Point(Float64(t), Float64(t)) for t in 0:9]

    # extract data: NamedPlot -> Series -> Points
    points = to_value.(plt.plots[1].plots[1].converted)[1][1]

    @test points ≈ expected
    @test f isa Figure
end

@testitem "namedplot with many colors" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=100)

    f = Figure()
    ax = Axis(f[1,1])
    p = namedplot!(ax, traj, :x)
    Legend(f[1,2], ax)
    # extract data: NamedPlot -> Series
    @test length(p.plots[1].plots[1].attributes.labels[]) == 100
    @test f isa Figure
end

@testitem "namedplot transform and merge" begin
    using CairoMakie
    traj = rand(NamedTrajectory, 10, state_dim=3)

    f = Figure()
    ax = Axis(f[1,1])
    p = namedplot!(ax, traj, :x, "y", x -> x .^ 2, linewidth=3, marker=:circle, merge=true)
    Legend(f[1,2], ax, merge=true)
    @test p.plots[1].attributes.labels[] == ["y" for i in 1:size(traj.x, 1)]

    ax = Axis(f[2,1])
    p = namedplot!(ax, traj, :x, "y", x -> x .^ 2, linewidth=3, marker=:circle)
    Legend(f[2,2], ax)
    @test p.plots[1].attributes.labels[] == ["y $i" for i in 1:size(traj.x, 1)]
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
    # check for all the right parts
    ax1, leg1, ax2, leg2, ax3, leg3 = f.content
    for ax in [ax1, ax2, ax3]
        @test ax isa Axis
    end
    for leg in [leg1, leg2, leg3]
        @test leg isa Legend
    end

    # test repeat label
    push!(transformations, (:x => x -> [x[2] ^6]))
    f = plot(
        traj, 
        transformations=transformations,
    )
    ax1, leg1, ax2, leg2, ax3, leg3, ax4, leg4 = f.content
    for ax in [ax1, ax2, ax3, ax4]
        @test ax isa Axis
    end
    for leg in [leg1, leg2, leg3, leg4]
        @test leg isa Legend
    end
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
    ax1, leg1, ax2, leg2 = f.content
    @test ax1.subtitle[] == transformation_titles[1]
    @test ax2.subtitle[] == transformation_titles[2]

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
    f = plot(theme_dark(), rand(NamedTrajectory, 10, state_dim=3))
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

end