module adv
dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/../sim") # simulation
push!(LOAD_PATH, dir_path * "/../vis") # visualization
push!(LOAD_PATH, dir_path * "/../adv") # adversary policies
push!(LOAD_PATH, dir_path * "/../dis") # discretization
push!(LOAD_PATH, dir_path * "/../pomdp")
using sim
using vis
using adv
using dis
using pomdp

using Serialization
using Printf
using LinearAlgebra

# include(dir * "/adv_weak_v2.jl")
# include(dir * "/adv_strong.jl")
# include(dir * "/adv_medium.jl")

export predict_collision

function replan_adv(agent_world::Pair{Agent, World}, t::Float64)
  world = agent_world.second
  agent_self = agent_world.first
  target_v = world.road.path.S[end] / 15 # target velocity is around the track in 15 seconds

  d_min = Inf
  c_type_closest = 0
  # Check for collisions with all other agents
  # If collisions are detected then get the closest one to the agent 
  for agent in world.agents
    if agent.id == agent_self.id
      continue
    end

    (d, t_c, c_type) = predict_collision(agent_self.x, agent_self, agent.x, agent, world, t)
    
    if c_type != 0 && d < d_min
      d_min = d
      c_type_closest = c_type
    end
  end

	return pomdp.sample_adv_a(agent.custom[5], c_type_closest)

end


function empty_controller!(x::AbstractArray{Float64},
													 u::AbstractArray{Float64},
													 dx::AbstractArray{Float64},
													 agent_world::Pair{Agent, World},
													 t::Float64)
	return
end

function parametrize(x::AbstractArray{Float64},
                        dx::AbstractArray{Float64},
                        world::World)

  (sx, sy) = sim.sp2sxsy(x[1], x[3], world.road.path)
	th = atan(sy, sx)

  s = mod(x[1], world.road.path.S[end])
  ds = x[2]
  dds = dx[2]

  p = x[3]
  dp = dx[3]

  v_x = ds*cos(th) + dp*sin(th)
  v_y = ds*sin(th) + dp*cos(th)

  a_x = dds*cos(th)
  a_y = dds*sin(th)

  (x_pos, y_pos) = sim.sp2xy(s, p, world.road.path)
  
  P = [x_pos v_x a_x/2;
       y_pos v_y a_y/2]

	return (P, th)
end


function closest_approach(p1::AbstractArray{Float64}, p2::AbstractArray{Float64})
  a = p1[1,1]
  b = p1[1,2]
  d = p1[2,1]
  y = p1[2,2]

  g = p2[1,1]
  h = p2[1,2]
  z = p2[2,1]
  k = p2[2,2]

  t = (-a*b + b*g + a*h - g*h + d*k - d*y - k*z + y*z) / (b^2 - 2*b*h + h^2 + k^2 - 2*k*y + y^2)

  v1 = [a + b*t, d + y*t]
  v2 = [g + h*t, z + k*t]

  return (v1, v2, t)
end

function predict_collision(x_self::Array{Float64}, agent_self::Agent,
													 x_opp::Array{Float64}, agent_opp::Agent,
													world::World,
													time::Float64=0.0)
	# Parametrize both agents
	# Compute closest approach
	# Compute Collision angle
	# Return r, t, c_th, collision_type
	# Predicted Collision Types:
	# 0 - No collision detected
	# 1 - agent_self crashes into agent_opp
	# 2 - agent_self is crashed into by agent opp
	
	# Compute dx for each agent
	dx_self = fill(0.0, 3)
	f_self = agent_self.controller!
	agent_self.controller! = empty_controller!
	agent_self.dynamics!(dx_self, x_self, Pair(agent_self, world), time)
	agent_self.controller! = f_self
	
	dx_opp = fill(0.0, 3)
	f_opp = agent_opp.controller!
	agent_opp.controller! = empty_controller!
	agent_opp.dynamics!(dx_opp, x_opp, Pair(agent_opp, world), time)
	agent_opp.controller! = f_opp

	# Parametrize both agents
	(P_self, th_self) = parametrize(dx_self, x_self, world)
	(P_opp, th_opp) = parametrize(dx_opp, x_opp, world)

	# Compute Closest approach collision
	(vc_self, vc_opp, t_c) = closest_approach(P_self, P_opp)

	c_v = vc_opp - vc_self

	# Compute collision angle
	th_collision = th_self - atan(c_v[2], c_v[1])
	
	# Determine collision type
	collision_type = 0

	if norm(c_v) < 10.0 && 0.0 < t_c < 1.5 # Predicted collision criteria
		if -pi / 2 < th_collision < pi / 2
			collision_type = HITTING
		else
			collision_type = BEING_HIT
		end
	end

	# Compute distance of self to collision
	d = norm(vc_self)

	return (d, t_c, collision_type)

end

end
