module StructKnotPoint

export KnotPoint

using ..StructNamedTrajectory

"""
    KnotPoint constructor
"""
struct KnotPoint{
    R <: Real,
    VT <: AbstractVector{R},
    CNames, CTypes <: StructNamedTrajectory.ComponentType,
    N <: Tuple{Vararg{Symbol}},
    CN <: Tuple{Vararg{Symbol}}
}
    k::Int
    data::VT
    timestep::R
    components::NamedTuple{CNames, CTypes}
    names::N
    control_names::CN
end

end