module StructKnotPoint

export KnotPoint

using ..StructNamedTrajectory

# TODO: Concerete type

"""
    KnotPoint constructor
"""
struct KnotPoint
    t::Int
    data::AbstractVector{Float64}
    timestep::Float64
    components::NamedTuple{
        cnames, <:Tuple{Vararg{AbstractVector{Int}}}
    } where cnames
    names::Tuple{Vararg{Symbol}}
    control_names::Tuple{Vararg{Symbol}}
end

end
