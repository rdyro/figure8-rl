# include("sim_types.jl")

function toy_controller!(u::AbstractArray{Float64},
                             x::AbstractArray{Float64},
                             dx::AbstractArray{Float64},
			     agent_world::Pair{Agent, World}, t::Float64)
  s_margin = 8 # one car length
	target_vel = 60 / 3.6
	
	agent = agent_world.first
	world = agent_world.second
	
	s = x[1]
	ds = dx[1]
	
	if ds < target_vel
		u[1] = 10
	else
		u[1] = 0
	end
	
	u[2] = -dx[3] # Counteract lateral motion
	
	for opponent in world.agents
		if opponent.id != agent.id && abs(opponent.x[1] - s) < s_margin && opponent.x[1] > s
			u[1] = -10
			u[2] = -dx[3] - 2
			if abs(opponent.x[3] - x[3]) > 1
				u[1] = 10
			end
			break;
		end
	end

	
	return
end
			 
