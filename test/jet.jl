# JET integrates tightly with the Julia compiler and is gated to v1.12.x only.
# On 1.10 / 1.11, JET 0.10 may not surface the same findings, which would
# break `broken = true` with "Unexpected Pass". On 1.13+ (nightly / pre)
# the compiler shifts faster than JET can track, so findings are noisy and
# unactionable until JET catches up. Don't widen this gate.
#
# Performance analysis (type instabilities / runtime dispatch) — run manually:
#     julia --project=. -e 'using NamedTrajectories, JET; JET.@report_opt rand(NamedTrajectory, 10)'

@testitem "JET correctness analysis" tags=[:jet] begin
    if v"1.12" <= VERSION < v"1.13"
        using JET, NamedTrajectories
        JET.test_package(NamedTrajectories; target_modules = (NamedTrajectories,))
    else
        @info "Skipping JET correctness analysis on Julia $VERSION (gated to 1.12.x)"
        @test true
    end
end
