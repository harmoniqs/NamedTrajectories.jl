module RandomTrajectories

using TestItems

using ..StructNamedTrajectory

"""
    rand(
        ::Type{NamedTrajectory},
        T::Int;
        timestep_value::Float64=1.0,
        timestep_name::Symbol=:Δt,
        free_time::Bool=false,
        timestep::Union{Float64,Symbol}=free_time ? timestep_name : timestep_value,
        state_dim::Int=3,
        control_dim::Int=2
    )

Create a random `NamedTrajectory` with `T` time steps, a state variable `x` of dimension 
`state_dim`, and a control variable `u` of dimension `control_dim`. If `free_time` is 
`true`, the time step is a symbol `timestep_name` and the time step value is 
`timestep_value`. Otherwise, the time step is a number `timestep_value`.
"""
function Base.rand(
    ::Type{NamedTrajectory},
    T::Int;
    timestep_value::Float64=1.0,
    timestep::Symbol=:Δt,
    state::Symbol=:x,
    control::Symbol=:u,
    state_dim::Int=3,
    control_dim::Int=2
)
    comps_data = (;
        state => randn(state_dim, T),
        control => randn(control_dim, T),
        timestep => fill(timestep_value, 1, T)
    )

    return NamedTrajectory(comps_data;  timestep=timestep, controls=(control, timestep))
end

# =========================================================================== #

@testitem "random trajectories" begin
    @test rand(NamedTrajectory, 5) isa NamedTrajectory
    @test rand(NamedTrajectory, 5; timestep=:dt).timestep == :dt
end

end
