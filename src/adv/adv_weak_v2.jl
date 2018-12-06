dir = @__DIR__
include_dependency(dir * "/../sim/sim.jl")

using sim
using adv

using Printf

function weak_controller!(u::AbstractArray{Float64},
														 x::AbstractArray{Float64},
														 dx::AbstractArray{Float64},
														 agent_world::Pair{Agent, World}, t::Float64)
	world = agent_world.second
	agent_self = agent_world.first
	target_v = world.road.path.S[end] / 15 # target velocity is around the track in 15 seconds

	d_min = Inf
	agent_pred_collision = -1
	c_type_closest = 0

	# If its not time to REPLAN then continue doing what you are doing
	if agent_self.custom.second > t
		if agent_self.custom.first == TRACKING
			u[1] = 0.0
		elseif agent_self.custom.first == BRAKING
			u[1] = 0.0
		elseif agent_self.custom.first == ACCELERATING
			u[1] = 0.0
		end
		u[2] = 0.0

		return
	end

	# Check for collisions with all other agents
	# If collisions are detected then get the closest one to the agent 
	for agent in world.agents
		if agent.id == agent_self.id
			continue
		end

		(c_type, d) = predict_collision(agent_self, agent, world, t)
		
		if c_type != 0 && d < d_min
			d_min = d
			agent_pred_collision = agent.id
			c_type_closest = c_type
		end
	end

	if c_type_closest != NO_COLLISION
		# u[1] = -5 * x[2] # Reduce acceleration by 20%
		agent_self.custom = (BRAKING, t + replan_time)
		u[1] = 0.0
		@printf("[COLLISION DETECTED] Distance: %d", d_min)
	else
		agent_self.custom = (TRACKING, t + replan_time)
		# u[1] = 0.1 * (target_v - x[2])
		u[1] = 0.0
		print("not colliding \n")
	end

	# u[2] = -1 * x[3] / dx[1]
	u[2] = 0.0

	return
end
