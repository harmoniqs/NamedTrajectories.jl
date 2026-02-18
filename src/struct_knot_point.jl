module StructKnotPoint

export KnotPoint

using ..StructNamedTrajectory

"""
    KnotPoint constructor
"""
struct KnotPoint{
    R<:Real,
    CNames,
    CTypes<:StructNamedTrajectory.ComponentType,
    N<:Tuple{Vararg{Symbol}},
    CN<:Tuple{Vararg{Symbol}},
}
    k::Int
    data::AbstractVector{R}
    timestep::R
    components::NamedTuple{CNames,CTypes}
    names::N
    control_names::CN
end

end
