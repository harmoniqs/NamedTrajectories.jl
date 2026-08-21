# Contributing to NamedTrajectories.jl

Welcome! NamedTrajectories.jl provides the high-performance trajectory data
structure used by [Piccolo.jl](https://github.com/harmoniqs/Piccolo.jl).
Licensed under MIT.

Every code change needs a GitHub issue and a PR. Create one first if it does
not exist.

## Dev Setup

```bash
git clone git@github.com:harmoniqs/NamedTrajectories.jl.git && cd NamedTrajectories.jl
julia --project -e 'using Pkg; Pkg.instantiate()'
```

## Running Tests

```bash
julia --project test/runtests.jl
```

## Submitting a PR

1. Create an issue describing the change.
2. Branch from `main`.
3. Commit focused changes with descriptive messages.
4. Run the test suite.
5. Open a draft PR with `Closes #<N>` at the first commit.
6. Mark ready when green.

## Getting Help

Use [GitHub Discussions](https://github.com/harmoniqs/NamedTrajectories.jl/discussions) for questions.
