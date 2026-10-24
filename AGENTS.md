# AGENTS.md — NamedTrajectories.jl

NamedTrajectories.jl is the trajectory datastructure layer for the
Piccolo stack: trajectories as named component arrays (drive components,
state components, per-component dimensions) over a shared time grid.

## Conventions

- `Pkg.test()` — Aqua + JET + runtests; standard Julia package layout.

## Notes for agents authoring against it

1. **Drive component names are API surface.** Downstream telemetry and
   pulse-emission code resolves the drive component by trying `:u` then
   `:a` — the component name is load-bearing across the stack, and a
   rename breaks downstream emitters (the amicode Run Inspector's live
   pulse plot among them). Treat component-name changes as breaking API
   changes.

## Provenance

Seeded 2026-08-22 as a minimal orientation seed under the
knowledge-routing law (insight-20260822-164700) — no stranded lessons
awaited routing at seed time. Extend via ADRs as decisions accrue.
