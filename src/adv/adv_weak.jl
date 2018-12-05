dir = @__DIR__
include_dependency(dir * "/../sim/sim.jl")

using sim
using adv

function weak_controller!(u::AbstractArray{Float64},
														 x::AbstractArray{Float64},
														 dx::AbstractArray{Float64},
														 agent_world::Pair{Agent, World}, t::Float64)
	world = agent_world.second
	P_self = parametrize(x, dx, world)
	target_v = world.road.path.S[end] / 15 # target velocity is around the track in 15 seconds

	collision_detected = false
	min_d = Inf
	min_t = Inf
	
	for agent in world.agents
		if agent.id == agent_world.first.id
			continue
		end

		dx_agent = fill(0.0, 3)
		f = agent.controller!
		agent.controller! = adv.empty_controller!
		agent.dynamics!(dx_agent, agent.x, Pair(agent, world), t)
		agent.controller! = f

		P_agent = parametrize(agent.x, dx_agent, world)

		(r, t_closest) = closest_approach(P_self, P_agent)
		d = norm([P_self[1,1] + P_self[1,2]*t_closest,
							P_self[2,1] + P_self[2,2]*t_closest])
		
		if r < 5.0 && 0.0 < t_closest < 5.0 # Criteria for detected collision
			collision_detected = true

			if d < min_d
				min_d = d
				min_t = t_closest
			end
		end
	end

	if collision_detected
		u[1] = -1 * x[2] # Reduce velocity by 20%
	else
		u[1] = 0.1 * (target_v - x[2])
	end

	u[2] = -1 * x[3] / dx[1]

	return
end
