@testitem "Aqua quality assurance" tags=[:aqua] begin
    using Aqua, NamedTrajectories

    Aqua.test_all(NamedTrajectories; deps_compat = (check_extras = false))
end
