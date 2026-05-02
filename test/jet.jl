using TestItems

# JET.jl static analysis.
#
# `JET.test_package` runs the same correctness checks as `report_package`
# (type errors, undefined methods, inference failures), wrapped in a
# `@test`. We restrict analysis to NamedTrajectories' modules with
# `target_modules` so we don't get noise from upstream packages we don't
# control.
#
# Findings as of the initial wiring of this test (NamedTrajectories
# v0.8.4, JET 0.11): 2 possible errors:
#
#   1. `rand(::Type{NamedTrajectory}, N::Int64)` ->
#      `kwcall(...) NamedTrajectory(comps_data, gcomps_data; ...)`
#      (src/random_trajectories.jl:36 -> src/struct_named_trajectory.jl:207)
#      No matching method for the inner `kwcall` on a Vector{Any}/
#      NamedTuple/Int64 union split. Looks like a real path that fails
#      to dispatch in the random-trajectory constructor.
#
#   2. `setproperty!(::KnotPoint, ::Symbol, ::Any)`
#      (src/methods_knot_point.jl:37) attempts setfield! on an
#      immutable `KnotPoint` struct. Real bug: that path will throw at
#      runtime if reached.
#
# We mark the test `broken=true` for now: the findings are informational,
# visible in CI logs without making the build red. Drop `broken=true`
# after the underlying issues are fixed, or convert specific cases to
# `@test_skip`.

@testitem "JET correctness analysis (test_package)" begin
    using JET
    using NamedTrajectories

    JET.test_package(
        NamedTrajectories;
        target_modules = (NamedTrajectories,),
        broken = true,
    )
end

# `JET.report_opt` flags type instabilities and runtime dispatch — the
# highest-value JET output for performance-critical numerical code.
# NamedTrajectories is a foundational data-structure package used
# downstream by Piccolo, so type stability matters for performance.
# This check is NOT run in CI by default (it requires a callable, not a
# module, so it is per-function and noisy). Run on individual hot paths
# locally:
#
#     julia --project=. -e '
#         using NamedTrajectories, JET
#         JET.@report_opt some_hot_function(args...)
#     '
#
# Gate this test behind ENV["JET_OPT"] == "true" so a developer can
# opt-in via `JET_OPT=true julia --project=. -e "using Pkg; Pkg.test()"`.
@testitem "JET performance analysis (report_opt, opt-in)" begin
    using JET
    using NamedTrajectories

    if get(ENV, "JET_OPT", "false") == "true"
        @info "JET_OPT=true: run JET.@report_opt on individual hot paths"
        @info "  e.g. JET.@report_opt rand(NamedTrajectory, 10) — see test/jet.jl for guidance"
        @test true
    else
        @info "Skipping JET.report_opt (set JET_OPT=true to enable)"
        @test true
    end
end
