module NamedTrajectories

using Reexport

@reexport using OrderedCollections: OrderedDict

include("struct_named_trajectory.jl")
@reexport using .StructNamedTrajectory

include("struct_knot_point.jl")
@reexport using .StructKnotPoint

include("utils.jl")
@reexport using .Utils

include("base_named_trajectory.jl")
@reexport using .BaseNamedTrajectory

include("methods_named_trajectory.jl")
@reexport using .MethodsNamedTrajectory

include("random_trajectories.jl")
@reexport using .RandomTrajectories

include("methods_knot_point.jl")
@reexport using .MethodsKnotPoint

include("plotting.jl")
@reexport using .Plotting

end
