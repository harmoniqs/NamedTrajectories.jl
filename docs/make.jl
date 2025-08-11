using NamedTrajectories

pages = [
    "Home" => "index.md",
    "Quickstart Guide" => "generated/quickstart.md",
    "Manual" => [
        "generated/man/constructors.md",
        "generated/man/params_in_struct.md",
        "generated/man/modifying.md"
    ],
    "Plotting" => "generated/plotting.md",
    "Library" => "lib.md"
]

# Check if utils.jl exists and warn if not found
utils_path = joinpath(@__DIR__, "utils.jl")
if !isfile(utils_path)
    error("docs/utils.jl is required but not found. Please run get_docs_utils.sh")
end

include("utils.jl")

generate_docs(
    @__DIR__,
    "NamedTrajectories",
    NamedTrajectories,
    pages;
    format_kwargs = (canonical = "https://docs.harmoniqs.co/NamedTrajectories.jl",),
)