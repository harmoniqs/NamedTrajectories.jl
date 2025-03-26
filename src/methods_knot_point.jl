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

function Base.getindex(slice::KnotPoint, symb::Symbol)
    if symb in fieldnames(KnotPoint)
        return getfield(slice, symb)
    else
        indices = slice.components[symb]
        return slice.data[indices]
    end
end



end
