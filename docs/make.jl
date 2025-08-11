using NamedTrajectories
using PiccoloDocsTemplate

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

generate_docs(
    @__DIR__,
    "NamedTrajectories",
    NamedTrajectories,
    pages;
    format_kwargs = (canonical = "https://docs.harmoniqs.co/NamedTrajectories.jl",),
)