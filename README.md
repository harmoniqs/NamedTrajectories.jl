<!--```@raw html-->
<div align="center">
  <a href="https://github.com/harmoniqs/Piccolo.jl">
    <img src="assets/logo.svg" alt="logo" width="25%"/>
  </a> 
</div>

<div align="center">
  <table>
    <tr>
      <td align="center">
        <b>Documentation</b>
        <br>
        <a href="https://docs.harmoniqs.co/NamedTrajectories/stable/">
          <img src="https://img.shields.io/badge/docs-stable-blue.svg" alt="Stable"/>
        </a>
        <a href="https://docs.harmoniqs.co/NamedTrajectories/dev/">
          <img src="https://img.shields.io/badge/docs-dev-blue.svg" alt="Dev"/>
        </a>
      </td>
      <td align="center">
        <b>Build Status</b>
        <br>
        <a href="https://github.com/harmoniqs/NamedTrajectories.jl/actions/workflows/CI.yml?query=branch%3Amain">
          <img src="https://github.com/harmoniqs/NamedTrajectories.jl/actions/workflows/CI.yml/badge.svg?branch=main" alt="Build Status"/>
        </a>
        <a href="https://codecov.io/gh/harmoniqs/NamedTrajectories.jl">
          <img src="https://codecov.io/gh/harmoniqs/NamedTrajectories.jl/branch/main/graph/badge.svg" alt="Coverage"/>
        </a>
      </td>
      <td align="center">
        <b>License</b>
        <br>
        <a href="https://opensource.org/licenses/MIT">
          <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="MIT License"/>
        </a>
      </td>
      <td align="center">
        <b>Support</b>
        <br>
        <a href="https://unitary.fund">
          <img src="https://img.shields.io/badge/Supported%20By-Unitary%20Fund-FFFF00.svg" alt="Unitary Fund"/>
        </a>
      </td>
    </tr>
  </table>
</div>

<div align="center">
  <i>An elegant way to handle messy trajectory data</i>
  <br>
</div>
<!--```-->

# NamedTrajectories.jl

**NamedTrajectories.jl** is a package for working with trajectories of named variables. It is designed to be used with the [Piccolo.jl](https://github.com/harmoniqs/Piccolo.jl) ecosystem.

## Installation

NamedTrajectories.jl is registered! Install in the REPL by entering pkg mode with `]` and then running 

```julia
pkg> add NamedTrajectories
```

### Features

- Abstract away messy indexing and vectorization details required for interfacing with numerical solvers.
- Easily handle multiple trajectories with different names, e.g. various states and controls.
- Simple plotting of trajectories.
- Provide a variety of helpful methods for common tasks.

## Basic Usage

Users can define `NamedTrajectory` types which have lots of useful functionality. For example, you can access the data by name or index.  In the case of an index, a `KnotPoint` is returned which contains the data for that timestep.

<!--```@example-->```julia
using NamedTrajectories

# define number of timesteps and timestep
T = 10
timestep=:dt

# build named tuple of components and data matrices
components = (
    x  = rand(3, T),
    u  = rand(2, T),
    dt = fill(0.1, T),
)

# build trajectory
traj = NamedTrajectory(components; timestep=timestep, controls=:u)

# access data by name
traj.x # returns 3x10 matrix of x data
traj.u # returns 2x10 matrix of u data

z1 = traj[1] # returns KnotPoint with x and u data

z1.x # returns 3 element vector of x data at timestep 1
z1.u # returns 2 element vector of u data at timestep 1

z1.dt # returns 10 element vector of timesteps

traj.data # returns data as 5x10 matrix
traj.names # returns names as tuple (:x, :u)
<!--```-->```

## Motivation

[NamedTrajectories.jl](https://github.com/harmoniqs/NamedTrajectories.jl) is designed to aid in the messy indexing involved in solving trajectory optimization problems of the form
```math
\begin{aligned}
    \arg \min_{\mathbf{Z}}\quad & J(\mathbf{Z}) \\
    \nonumber \text{s.t.}\qquad & \mathbf{f}(\mathbf{Z}) = 0 \\
    \nonumber & \mathbf{g}(\mathbf{Z}) \le 0  
\end{aligned}
```
where $\mathbf{Z}$ is a trajectory.

In more detail, this problem might look something like
```math
\begin{align*}
\underset{u^1_{1:T}, \dots, u^{n_c}_{1:T}}{\underset{x^1_{1:T}, \cdots, x^{n_s}_{1:T}}{\text{minimize}}} &\quad J \left(x^{1:n_s}_{1:T},u^{1:n_c}_{1:T} \right) \\
\text{subject to} & \quad f \left(x^{1:n_s}_{1:T},u^{1:n_c}_{1:T} \right) = 0 \\
& \quad x^i_1 = x^i_{\text{initial}} \\
& \quad x^i_T = x^i_{\text{final}} \\
& \quad u^i_1 = u^i_{\text{initial}} \\
& \quad u^i_T = u^i_{\text{final}} \\
& \quad x^i_{\min} < x^i_t < x^i_{\max} \\
& \quad u^i_{\min} < u^i_t < u^i_{\max} \\
\end{align*}
```
where $x^i_t$ is the $i$ th state variable and $u^i_t$ is the $i$ th control variable at timestep $t$; state and control variables can be of arbitrary dimension. The function $f$ is a nonlinear constraint function and $J$ is the objective function. These problems can have an arbitrary number of state ($n_s$) and control ($n_c$) variables, and the number of timesteps $T$ can vary as well.  

It is common practice in trajectory optimization to bundle all of the state and control variables together into a single *knot point*

```math
z_t = \begin{pmatrix}
    x^1_t \\
    \vdots \\
    x^{n_s}_t \\
    u^1_t \\
    \vdots \\
    u^{n_c}_t
  \end{pmatrix}.
```

The trajectory optimization problem can then be succinctly written as

```math
\begin{align*}
\underset{z_{1:T}}{\text{minimize}} &\quad J \left(z_{1:T} \right) \\
\text{subject to} & \quad f \left(z_{1:T} \right) = 0 \\
& \quad z_1 = z_{\text{initial}} \\
& \quad z_T = z_{\text{final}} \\
& \quad z_{\min} < z_t < z_{\max} \\
\end{align*}
```

The `NamedTrajectories` package provides a `NamedTrajectory` type which abstracts away the messy indexing and vectorization details required for interfacing with numerical solvers.  It also provides a variety of helpful methods for common tasks.  For example, you can access the data by name or index.  In the case of an index, a `KnotPoint` is returned which contains the data for that timestep.


### Building Documentation
This package uses a Documenter config that is shared with many of our other repositories. To build the docs, you will need to run the docs setup script to clone and pull down the utility. 
```
# first time only
./docs/get_docs_utils.sh   # or ./get_docs_utils.sh if cwd is in ./docs/
```

To build the docs pages:
```
julia --project=docs docs/make.jl
```

or editing the docs live:
```
julia --project=docs
> using LiveServer, Piccolo, Revise
> servedocs(literate_dir="docs/literate", skip_dirs=["docs/src/generated", "docs/src/assets/"], skip_files=["docs/src/index.md"])
```

> **Note:** `servedocs` needs to watch a subset of the files in the `docs/` folder. If it watches files that are generated on a docs build/re-build, `servedocs` will continuously try to re-serve the pages.
> 
> To prevent this, ensure all generated files are included in the skip dirs or skip files args for `servedocs`.


For example, if we forget index.md like so:
```
julia --project=docs
> using LiveServer, Piccolo, Revise
> servedocs(literate_dir="docs/literate", skip_dirs=["docs/src/generated", "docs/src/assets/"])
```
it will not build and serve.

-----

*"It seems that perfection is attained not when there is nothing more to add, but when there is nothing more to take away." - Antoine de Saint-Exupéry*
