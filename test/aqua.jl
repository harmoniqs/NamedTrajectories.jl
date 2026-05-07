@testitem "Aqua quality assurance" tags=[:aqua] begin
    using Aqua, NamedTrajectories

    Aqua.test_all(
        NamedTrajectories;
        # TODO: drop ComputePipeline + LaTeXStrings; move TestItemRunner to [extras]
        stale_deps = (ignore = [:ComputePipeline, :TestItemRunner, :LaTeXStrings],),
        deps_compat = (check_extras = false, ignore = [:Random]),
    )
end
