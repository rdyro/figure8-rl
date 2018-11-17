# include("sim_types.jl")

function controller_toy!(u::AbstractArray{Float64},
                             x::AbstractArray{Float64},
                             dx::AbstractArray{Float64},
			     agent_world::Pair{Agent, World}, t::Float64)
  s_margin = 4.5 # one car length
	target_vel = 60 / 3.6

	world = agent_world.first
	
	s = x[1]
	ds = dx[1]
	
	if ds < target_vel
		u[1] = 10
	else
		u[1] = 0
	end
	
	for opponent in world.agents
		if opponent.x[1] - s < s_margin
			u[1] = -10
			break;
		end
	end

	u[2] = -dx[3] # Counteract lateral motion
	
	return
end
			 
