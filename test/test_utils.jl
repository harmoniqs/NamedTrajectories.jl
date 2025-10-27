using Random

function get_free_time_traj(;
    N::Int=5, 
    Δt::Symbol=:Δt, 
    x_dim::Int=3, 
    a_dim::Int=2,
    kwargs...
)
    free_time_data = (x = rand(x_dim, N), u = rand(a_dim, N), Δt = rand(1, N))
    return NamedTrajectory(free_time_data; timestep=Δt, controls=:u, kwargs...)
end

function named_trajectory_type_1(; free_time=false)
    # Hadamard gate, two dda controls (random), Δt = 0.2
    data = [
        1.0          0.957107     0.853553     0.75         0.707107;
        0.0          0.103553     0.353553     0.603553     0.707107;
        0.0          0.103553     0.146447     0.103553     1.38778e-17;
        0.0         -0.25        -0.353553    -0.25        -1.52656e-16;
        0.0          0.103553     0.353553     0.603553     0.707107;
        1.0          0.75         0.146447    -0.457107    -0.707107;
        0.0         -0.25        -0.353553    -0.25        -1.249e-16;
        0.0          0.603553     0.853553     0.603553     4.16334e-16;
        0.0         -0.243953     0.959151    -0.665253     0.0;
        0.0          0.0139165    0.668917     0.625329     0.0;
        0.00393491   0.0240775   -0.00942396   0.00329391   0.00941354;
       -0.00223794  -0.0105816    0.00328457   0.0204239    0.0253415;
        0.0058186    0.00686586  -0.00422555   0.00442631   0.000319156;
       -0.00134597  -0.00120682   0.0114915    0.00189333  -0.0251649;
        0.2          0.2          0.2          0.2          0.2
    ]

    if free_time
        components = (
            Ũ⃗ = data[1:8, :],
            a = data[9:10, :],
            da = data[11:12, :],
            dda = data[13:14, :],
            Δt = data[15:15, :]
        )
        controls = (:dda, :Δt)
        timestep = :Δt
        bounds = (
            a = ([-1.0, -1.0], [1.0, 1.0]), 
            dda = ([-1.0, -1.0], [1.0, 1.0]), 
            Δt = ([0.1], [0.30000000000000004])
        )
    else 
        components = (
            Ũ⃗ = data[1:8, :],
            a = data[9:10, :],
            da = data[11:12, :],
            dda = data[13:14, :]
        )
        controls = (:dda,)
        timestep = 0.2
        bounds = (
            a = ([-1.0, -1.0], [1.0, 1.0]), 
            dda = ([-1.0, -1.0], [1.0, 1.0]), 
        )
    end

    initial = (
        Ũ⃗ = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        a = [0.0, 0.0]
    )
    final = (a = [0.0, 0.0],)
    goal = (Ũ⃗ = [0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],)

    return NamedTrajectory(
        components;
        controls=controls,
        timestep=timestep,
        bounds=bounds,
        initial=initial,
        final=final,
        goal=goal
    )
end
