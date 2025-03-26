module MethodsKnotPoint

using ..StructKnotPoint

function Base.getproperty(slice::KnotPoint, symb::Symbol)
    if symb in fieldnames(KnotPoint)
        return getfield(slice, symb)
    else
        indices = slice.components[symb]
        return view(slice.data, indices)
    end
end

function Base.getindex(slice::KnotPoint, symb::Symbol) # is this method redundant? i.e. should we just dispatch to getproperty?
    if symb in fieldnames(KnotPoint)
        return getfield(slice, symb)
    else
        indices = slice.components[symb]
        return view(slice.data, indices)
    end
end

function Base.setproperty!(slice::KnotPoint, symb::Symbol, val::Any)
    if symb in fieldnames(KnotPoint)
        setfield!(slice, symb, val) # will throw error since KnotPoint is an immutable struct
    else
        update!(slice, symb, val)
    end
end

function Base.setindex!(slice::KnotPoint, symb::Symbol, val::Any)
    setproperty!(slice, symb, val)
end


function update!(slice::KnotPoint, symb::Symbol, data::AbstractVector{Float64})
    @assert symb in slice.names
    @assert size(data, 1) == (slice.components[symb].stop - slice.components[symb].start + 1)

    Base.getproperty(slice, symb)[:] = data
    return nothing
end


end
