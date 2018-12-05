dir = @__DIR__
include_dependency(dir * "/../sim/sim.jl")

using sim
using adv

function strong_controller!(u::AbstractArray{Float64},
														 x::AbstractArray{Float64},
														 dx::AbstractArray{Float64},
														 agent_world::Pair{Agent, World}, t::Float64)
	world = agent_world.second
	target_v = world.road.path.S[end] / 15
	u[1] = 0.1 * (target_v - x[2])
	u[2] = -1*x[3] / dx[1]

	return
end
