# Performance analysis (type instabilities / runtime dispatch) — run manually:
#     julia --project=. -e 'using NamedTrajectories, JET; JET.@report_opt rand(NamedTrajectory, 10)'

@testitem "JET correctness analysis" tags=[:jet] begin
    using JET, NamedTrajectories
    JET.test_package(NamedTrajectories; target_modules = (NamedTrajectories,), broken = true)  # TODO: 2 findings
end
