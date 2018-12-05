dir = @__DIR__
include_dependency(dir * "/../sim/sim.jl")

using sim
using adv

function meek_controller!(u::AbstractArray{Float64},
														 x::AbstractArray{Float64},
														 dx::AbstractArray{Float64},
														 agent_world::Pair{Agent, World}, t::Float64)
	world = agent_world.second
	P_self = parametrize(x, dx, world)
	target_v = world.road.path.S[end] / 15 # target velocity is around the track in 15 seconds

	collision_detected = false
	
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

		(dist, t_closest) = closest_approach(P_self, P_agent)
		
		if dist < 5.0 && 0.0 < t_closest < 5.0 # Criteria for detected collision
			collision_detected = true
			break
		end
	end

	if collision_detected
		if x[2] > 0.0
			u[1] = -0.7 * x[2] # Reduce velocity by 20%
				
		else
			u[1] = 0.2 * x[2] # For if car is going backwards idk why it would
		end
	else
		u[1] = 0.1 * (target_v - x[2])
	end

	u[2] = -1*x[3] / dx[1]

	return
end
