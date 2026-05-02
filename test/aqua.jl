using TestItems

# Aqua.jl runs the standard package-hygiene checks: ambiguities,
# unbound_args, undefined_exports, project_extras, stale_deps,
# deps_compat, piracies, persistent_tasks.
#
# Findings as of the initial wiring of this test (NamedTrajectories
# v0.8.4, Aqua 0.8):
#
#   - ambiguities:      0   (clean)
#   - piracies:         0   (clean)
#   - stale_deps:       3   (ComputePipeline, TestItemRunner,
#                            LaTeXStrings — listed in [deps] but the
#                            module never imports them; see below)
#   - deps_compat:      1   (Random: stdlib, missing compat)
#                          + extras (CairoMakie, Test) skipped via kwarg
#   - project_extras:   0   (clean)
#   - undefined_exports:0   (clean)
#   - unbound_args:     0   (clean)
#   - persistent_tasks: 0   (clean)
#
# We mark the failing categories `broken=true` so CI stays green while
# the findings are visible in the log. Each disable / broken has a TODO
# for follow-up cleanup. DO NOT add new entries here without an issue
# link or a removal date.

@testitem "Aqua quality assurance" begin
    using Aqua
    using NamedTrajectories

    Aqua.test_all(
        NamedTrajectories;
        # 3 stale deps flagged: ComputePipeline, TestItemRunner,
        # LaTeXStrings appear in [deps] but the package source never
        # `using`s or `import`s them. TestItemRunner is only needed by
        # the test harness and likely belongs in [extras]; LaTeXStrings
        # may be a transitive of Makie used only inside the
        # PlottingExt extension; ComputePipeline appears unused.
        # `stale_deps` does NOT accept `broken=true` (only `ignore=`),
        # so we ignore the three known-stale entries here. Any *new*
        # stale dep will still fail the check.
        # TODO: move/drop these and remove the ignore list.
        stale_deps = (ignore = [:ComputePipeline, :TestItemRunner, :LaTeXStrings],),
        # `Random` is a stdlib (Aqua flags it as missing compat);
        # CairoMakie/Test are test-only extras and most packages don't
        # compat-bound them. We mark this `broken=true` so the finding
        # is visible without failing CI. TODO: decide a project policy
        # on stdlib + test-extras compat and either add the entries or
        # re-enable with a narrowed `ignore`.
        deps_compat = (check_extras = false, broken = true),
    )
end
