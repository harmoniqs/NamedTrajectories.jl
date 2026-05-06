module Utils

export save
export load_traj
export derivative
export integral

using JLD2

using ..StructNamedTrajectory
using ..StructKnotPoint

function JLD2.save(filename::String, traj::NamedTrajectory)
    @assert split(filename, ".")[end] == "jld2"
    save(filename, "traj", traj)
end

function load_traj(filename::String)
    @assert split(filename, ".")[end] == "jld2"
    return load(filename, "traj")
end

"""
    derivative(X::AbstractMatrix, Δt::AbstractVecOrMat)
    derivative(X::AbstractMatrix, Δt::Float64)

Compute the derivative of the data matrix `X`.
"""
function derivative(X::AbstractMatrix, Δt::AbstractVecOrMat)
    if Δt isa AbstractMatrix
        @assert size(Δt, 1) == 1 "X must be a row vector if Δt is a matrix"
        Δt = Δt[1, :]
    end
    @assert size(X, 2) == length(Δt) "number of columns of X ($(size(X, 2))) must equal length of Δt ($(length(Δt)))"
    dX = zeros(eltype(X), size(X))

    dX[:, 1] = (X[:, 2] - X[:, 1]) / Δt[1]

    for t in axes(X, 2)[2:(end-1)]
        Δx = X[:, t+1] - X[:, t]
        h = Δt[t]
        dX[:, t] = Δx / h
    end

    dX[:, end] = dX[:, end-1]
    return dX
end

derivative(X::AbstractMatrix, Δt::Float64) = derivative(X, fill(Δt, size(X, 2)))


"""
    integral(X::AbstractMatrix, Δt::AbstractVector)
    integral(X::AbstractMatrix, Δt::Float64)

Compute the integral of the data matrix `X`.
"""
function integral(X::AbstractMatrix, Δt::AbstractVector)
    ∫X = similar(X)
    ∫X[:, 1] = zeros(size(X, 1))
    for t in axes(X, 2)[2:end]
        # trapezoidal rule
        ∫X[:, t] = ∫X[:, t-1] + (X[:, t] + X[:, t-1])/2 * Δt[t-1]
    end
    return ∫X
end

integral(X::AbstractMatrix, Δt::Float64) = integral(X, fill(Δt, size(X, 2)))


# =========================================================================== #

using TestItems

@testitem "save and load_traj round-trip" begin
    traj = rand(NamedTrajectory, 5)
    mktempdir() do dir
        path = joinpath(dir, "traj.jld2")
        save(path, traj)
        @test isfile(path)
        loaded = load_traj(path)
        @test isequal(loaded, traj)
    end
end

@testitem "save/load filename extension assertion" begin
    traj = rand(NamedTrajectory, 5)
    @test_throws AssertionError save("bad_extension.txt", traj)
    @test_throws AssertionError load_traj("missing.txt")
end

@testitem "derivative: vector and Float64 timesteps agree on uniform grid" begin
    X = [1.0 2.0 4.0 7.0 11.0; 0.0 1.0 4.0 9.0 16.0]
    Δt_vec = fill(0.5, size(X, 2))
    dX_vec = derivative(X, Δt_vec)
    @test size(dX_vec) == size(X)
    @test dX_vec[:, 1] ≈ (X[:, 2] - X[:, 1]) / 0.5
    @test dX_vec[:, end] ≈ dX_vec[:, end-1]

    dX_float = derivative(X, 0.5)
    @test dX_float ≈ dX_vec
end

@testitem "derivative: matrix Δt with single row collapses to vector" begin
    X = randn(3, 6)
    Δt_row = reshape(fill(0.25, 6), 1, 6)
    dX_mat = derivative(X, Δt_row)
    dX_vec = derivative(X, vec(Δt_row))
    @test dX_mat ≈ dX_vec
end

@testitem "integral: vector and Float64 timesteps agree on uniform grid" begin
    X = [1.0 2.0 3.0 4.0 5.0; 0.0 1.0 2.0 3.0 4.0]
    Δt_vec = fill(1.0, size(X, 2))
    ∫X = integral(X, Δt_vec)
    @test size(∫X) == size(X)
    @test ∫X[:, 1] == zeros(size(X, 1))
    @test ∫X[:, 2] ≈ (X[:, 1] + X[:, 2]) / 2 * 1.0
    @test ∫X[:, 3] ≈ ∫X[:, 2] + (X[:, 2] + X[:, 3]) / 2 * 1.0

    ∫X_const = integral(X, 1.0)
    @test ∫X_const ≈ ∫X
end

end
