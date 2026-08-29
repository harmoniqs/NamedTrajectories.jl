module TimeWarp

using ForwardDiff
using TestItems

export AbstractTimeWarp, GlobalScale, PiecewiseLinearWarp
export warp_params, n_params, with_params
export deltats, deltats_row, knot_times, duration, is_monotone
export ddeltats_dparams

# ---------------------------------------------------------------------------- #
# AbstractTimeWarp — monotone time warps on the normalized knot lattice
# ---------------------------------------------------------------------------- #

"""
    AbstractTimeWarp

A monotone time warp `w : [0,1] → [0, T]` applied to the normalized coordinates of a
trajectory's uniform knot lattice (`N` knots at `sₖ = (k-1)/(N-1)`). Under a warp, a
`NamedTrajectory`'s timestep rows are **derived** — `Δtₖ = w(sₖ₊₁) - w(sₖ)` — and are
never decision data: they stay present as components but are excluded from the packed
vector (see `NamedTrajectory(...; warp = ...)`).

Interface contract for a concrete warp `W <: AbstractTimeWarp`:

  - `(w::W)(s::Real)::Real` — monotone; must accept `ForwardDiff.Dual` in `s` *and* in params;
  - `warp_params(w)::AbstractVector` — current parameter values (may be empty);
  - `n_params(w)::Int`;
  - `with_params(w, θ)::AbstractTimeWarp` — Dual-generic rebuild (the AD seam);
  - `duration(w)::Real` — `== w(1) - w(0)`.

`deltats`, `deltats_row`, `knot_times`, `is_monotone` and `ddeltats_dparams` are derived
generically from that contract.
"""
abstract type AbstractTimeWarp end

_check_lattice(N::Int) =
    N ≥ 2 || throw(ArgumentError("a time warp needs at least 2 knots (got N = $N)"))

"""
    deltats(w, N::Int)::Vector

Physical duration of each of the `N - 1` lattice intervals under the time warp `w`:
`deltats(w, N)[k] = w(k/(N-1)) - w((k-1)/(N-1))`. Generic over `w`'s parameter types
(`Float64`, `ForwardDiff.Dual`, …).
"""
function deltats(w, N::Int)
    _check_lattice(N)
    L = N - 1
    return [w(k / L) - w((k - 1) / L) for k = 1:L]
end

"""
    deltats_row(w, N::Int)::Vector

The derived timestep row of length `N`: [`deltats`](@ref) padded with its final value —
the Piccolo convention (`get_times` consumes only the first `N - 1` entries).
"""
function deltats_row(w, N::Int)
    dts = deltats(w, N)
    return [dts; dts[end]]
end

"""
    knot_times(w, N::Int)::Vector

Physical times of the `N` lattice knots under the time warp `w`:
`knot_times(w, N)[k] = w((k-1)/(N-1))`. Consistent with `get_times` on a trajectory
whose timestep rows are derived from `w`.
"""
function knot_times(w, N::Int)
    _check_lattice(N)
    L = N - 1
    return [w((k - 1) / L) for k = 1:N]
end

"""
    duration(w::AbstractTimeWarp)::Real

Total physical duration of the warp: `w(1) - w(0)`.
"""
duration(w::AbstractTimeWarp) = w(1.0) - w(0.0)

"""
    is_monotone(w, N::Int)::Bool

Monotonicity of the time warp `w` over the `N`-knot lattice — sufficient (and necessary,
for a continuous warp) for all derived timesteps to be positive.
"""
is_monotone(w, N::Int) = all(>(0), deltats(w, N))

"""
    ddeltats_dparams(w, N::Int)::Matrix

Chain rule `∂Δtₖ/∂θⱼ` as an `(N - 1) × n_params(w)` matrix, where `θ` are the warp
parameters ([`warp_params`](@ref)). The generic implementation differentiates
[`deltats`](@ref) through [`with_params`](@ref) with ForwardDiff; concrete warps may
override it exactly (as `GlobalScale` does — no floating subtraction).
"""
function ddeltats_dparams(w, N::Int)
    return ForwardDiff.jacobian(θ -> deltats(with_params(w, θ), N), warp_params(w))
end

# ---- GlobalScale: w(s) = T·s — the single-scalar warp (v1) ------------------- #

"""
    GlobalScale(T)

Global time scaling `w(s) = T·s`. `T` is the single warp parameter — the total
duration, and (downstream) the scalar decision variable that replaces the per-knot
`Δt` block of a free-time formulation. Parametric in `R` so that `with_params` under
ForwardDiff produces a `Dual`-typed warp. The derived timesteps are
`Δtₖ = T·wₖ` with the **exact rational** lattice weights `wₖ = 1/(N-1)`;
[`ddeltats_dparams`](@ref) exposes `∂Δtₖ/∂T = wₖ` exactly.
"""
struct GlobalScale{R<:Real} <: AbstractTimeWarp
    T::R
end

(w::GlobalScale)(s::Real) = w.T * s

warp_params(w::GlobalScale) = [w.T]
n_params(::GlobalScale) = 1
with_params(::GlobalScale, θ::AbstractVector{<:Real}) = GlobalScale(θ[1])
duration(w::GlobalScale) = w.T

# Exact: dΔtₖ/dT is the interval's lattice fraction 1/(N-1) — no floating subtraction.
function ddeltats_dparams(w::GlobalScale, N::Int)
    _check_lattice(N)
    return reshape(fill(Float64(1 // (N - 1)), N - 1), :, 1)
end

# ---- PiecewiseLinearWarp: interface-complete, NOT NLP-wired (v1) -------------- #

"""
    PiecewiseLinearWarp(anchors, durations)

Piecewise-linear monotone warp: `anchors` are exact normalized breakpoints
`0 = a₁ < … < a_K = 1` (`Rational{Int}`, on the lattice), and `durations[j] > 0` is the
physical duration of `[aⱼ, aⱼ₊₁]`. The parameters are the `durations`; positivity makes
the warp monotone. Interface-tested in v1 but **not wired** into the packed-vector
optimization flow.
"""
struct PiecewiseLinearWarp{R<:Real} <: AbstractTimeWarp
    anchors::Vector{Rational{Int}}
    durations::Vector{R}

    function PiecewiseLinearWarp(
        anchors::AbstractVector{<:Rational{<:Integer}},
        durations::AbstractVector{R},
    ) where {R<:Real}
        length(anchors) ≥ 2 || throw(ArgumentError("need at least 2 anchors"))
        first(anchors) == 0 || throw(ArgumentError("first anchor must be 0"))
        last(anchors) == 1 || throw(ArgumentError("last anchor must be 1"))
        for j = 1:(length(anchors)-1)
            anchors[j] < anchors[j+1] ||
                throw(ArgumentError("anchors must be strictly increasing at index $j"))
        end
        length(durations) == length(anchors) - 1 ||
            throw(ArgumentError("need length(durations) == length(anchors) - 1"))
        return new{R}(Vector{Rational{Int}}(anchors), Vector{R}(durations))
    end
end

function (w::PiecewiseLinearWarp)(s::Real)
    a = w.anchors
    j = clamp(searchsortedlast(a, s), 1, length(a) - 1)
    t = sum(view(w.durations, 1:(j-1)); init = zero(eltype(w.durations)))
    return t + w.durations[j] * ((s - a[j]) / (a[j+1] - a[j]))
end

warp_params(w::PiecewiseLinearWarp) = collect(w.durations)
n_params(w::PiecewiseLinearWarp) = length(w.durations)
with_params(w::PiecewiseLinearWarp, θ::AbstractVector{<:Real}) =
    PiecewiseLinearWarp(w.anchors, θ)
duration(w::PiecewiseLinearWarp) = sum(w.durations)

# =========================================================================== #

@testitem "GlobalScale: exact lattice weights and chain-rule override" begin
    using NamedTrajectories
    import ForwardDiff

    w = GlobalScale(7.5)
    N = 13  # L = 12 intervals

    @test duration(w) == 7.5
    @test knot_times(w, N) ≈ 7.5 .* (0:12) ./ 12
    @test deltats(w, N) ≈ fill(7.5 / 12, 12)
    @test sum(deltats(w, N)) ≈ duration(w)
    @test is_monotone(w, N)
    @test !is_monotone(GlobalScale(0.0), N)
    @test !is_monotone(GlobalScale(-1.0), N)

    # the padded row: length N, first N-1 entries are the interval durations
    row = deltats_row(w, N)
    @test length(row) == N
    @test row[1:(end-1)] == deltats(w, N)
    @test row[end] == row[end-1]

    # Exact override: the column IS the rational lattice weights — exact ==, never ≈.
    J = ddeltats_dparams(w, N)
    @test size(J) == (12, 1)
    @test J[:, 1] == Float64.(fill(1 // 12, 12))

    # Agreement with the generic ForwardDiff path (invoked explicitly).
    J_fd = ForwardDiff.jacobian(θ -> deltats(with_params(w, θ), N), warp_params(w))
    @test J ≈ J_fd

    # AD seam: T flows through with_params as a Dual.
    @test ForwardDiff.derivative(T -> GlobalScale(T)(0.4), 2.0) ≈ 0.4
    @test warp_params(with_params(w, [3.25])) == [3.25]
    @test n_params(w) == 1
end

@testitem "PiecewiseLinearWarp interface contract" begin
    using NamedTrajectories
    import ForwardDiff

    anchors = [0 // 1, 1 // 3, 1 // 2, 1 // 1]
    w = PiecewiseLinearWarp(anchors, [1.0, 2.0, 3.0])

    # Evaluation: exact at anchors, linear between.
    @test w(0.0) == 0.0
    @test w(1 / 3) ≈ 1.0
    @test w(1 / 2) ≈ 3.0
    @test w(1.0) ≈ 6.0
    @test w(1 / 6) ≈ 0.5
    @test w(0.75) ≈ 4.5
    @test duration(w) ≈ 6.0

    # Derived Δt on an anchor-aligned lattice (L = 6 ⇒ N = 7 knots):
    # [0,1/3] → 1.0, [1/3,1/2] → 2.0, [1/2,1] → 3.0.
    @test deltats(w, 7) ≈ [0.5, 0.5, 2.0, 1.0, 1.0, 1.0]
    @test sum(deltats(w, 7)) ≈ duration(w)
    @test knot_times(w, 7) ≈ [0.0, 0.5, 1.0, 3.0, 4.0, 5.0, 6.0]
    @test is_monotone(w, 7)
    @test !is_monotone(PiecewiseLinearWarp(anchors, [1.0, -2.0, 3.0]), 7)

    # Chain rule via the generic ForwardDiff path, checked against hand-computed rows:
    # [0,1/3] splits into two intervals of weight 1/2; [1/3,1/2] is ONE interval
    # covering the whole segment, so ∂Δt₃/∂θ₂ = 1; [1/2,1] spans three intervals
    # of lattice fraction 1/3, so each carries ∂Δt/∂θ₃ = 1/3.
    @test ddeltats_dparams(w, 7) ≈
          [0.5 0.0 0.0; 0.5 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1/3; 0.0 0.0 1/3; 0.0 0.0 1/3]

    # Params round-trip + Dual-genericity of the rebuild.
    @test warp_params(w) == [1.0, 2.0, 3.0]
    @test n_params(w) == 3
    @test warp_params(with_params(w, [4.0, 5.0, 6.0])) == [4.0, 5.0, 6.0]
    grad = ForwardDiff.gradient(θ -> with_params(w, θ)(0.75), warp_params(w))
    @test grad ≈ [1.0, 1.0, 0.5]

    # Constructor validation.
    @test_throws ArgumentError PiecewiseLinearWarp([0 // 1, 1 // 2], [1.0, 1.0]) # length mismatch
    @test_throws ArgumentError PiecewiseLinearWarp([1 // 3, 1 // 1], [1.0])      # first ≠ 0
    @test_throws ArgumentError PiecewiseLinearWarp([0 // 1, 1 // 2], [1.0])      # last ≠ 1
    @test_throws ArgumentError PiecewiseLinearWarp(
        [0 // 1, 1 // 2, 1 // 3, 1 // 1],
        [1.0, 1.0, 1.0],
    )                                                                             # not increasing
end

@testitem "warp lattice functions validate N ≥ 2" begin
    using NamedTrajectories

    w = GlobalScale(1.0)
    @test_throws ArgumentError deltats(w, 1)
    @test_throws ArgumentError deltats_row(w, 1)
    @test_throws ArgumentError knot_times(w, 1)
    @test_throws ArgumentError is_monotone(w, 1)
    @test_throws ArgumentError ddeltats_dparams(w, 1)
end

end
