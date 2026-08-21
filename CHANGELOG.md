# Changelog

All notable changes to NamedTrajectories.jl are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the package
follows Julia 0.x SemVer.

## [0.9.3] — 2026-08-20

### Fixed

- **`KnotPoint` `setindex!` had its arguments transposed** against Base's
  `setindex!(collection, value, key)` convention — the method could never
  match the `kp[:name] = val` lowering and was dead code since introduction.
  Now `setindex!(slice, val, symb)`; the syntax works, and setting a struct
  field raises the intended descriptive error. Found by the 100%-coverage
  push (#151).

### Added

- `update!` widened from `AbstractVector{Float64}`/`AbstractMatrix{Float64}`
  to `{<:Real}` — non-Float64 (Float32, Int) data converts into the
  trajectory's Float64 storage (#149). First step toward non-Float64/device
  trajectories.

### Removed

- Three provably-unreachable code paths (the coverage push's archaeology,
  #151): the `BoundsError` guard in `merge` (eachindex indices are always
  valid), the vestigial numeric-`timestep` branches of
  `get_times`/`get_timesteps` (the field is `::Symbol` by construction), and
  the "no components found with suffix" error in `get_suffix`.

### Changed

- `get_bounds_from_dims` validates malformed bounds with a descriptive
  `ArgumentError` (previously a bare `TypeError` from dict conversion, and
  an `@assert` that compiled away) (#151).

### Housekeeping

- Notation pass: "knot points" for N everywhere in prose, "timestep"
  reserved for Δt (#147). Test coverage brought to **100% line coverage**
  (#151).
