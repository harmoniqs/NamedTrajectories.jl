# JET integrates tightly with the Julia compiler and is gated to v1.12 only.
# On 1.10 / 1.11, JET 0.10 may not surface the same findings, which would
# break `broken = true` with "Unexpected Pass". Don't relax this gate.
#
# Performance analysis (type instabilities / runtime dispatch) — run manually:
#     julia --project=. -e 'using NamedTrajectories, JET; JET.@report_opt rand(NamedTrajectory, 10)'

@testitem "JET correctness analysis" tags=[:jet] begin
    if VERSION >= v"1.12"
        using JET, NamedTrajectories
        JET.test_package(
            NamedTrajectories;
            target_modules = (NamedTrajectories,),
            broken = true,
        )  # TODO: 2 findings
    else
        @info "Skipping JET correctness analysis on Julia $VERSION (requires >= 1.12)"
        @test true
    end
end
