module adv
dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/../sim") # simulation
push!(LOAD_PATH, dir_path * "/../vis") # visualization
push!(LOAD_PATH, dir_path * "/../dis") # discretization
push!(LOAD_PATH, dir_path * "/../pomdp")
using sim
using vis
using dis
using pomdp

using Serialization
using Printf
using LinearAlgebra

# include(dir * "/adv_weak_v2.jl")
# include(dir * "/adv_strong.jl")
# include(dir * "/adv_medium.jl")

export predict_collision

function replan_adv(agent_self::Agent, world::World, t::Float64=0.0)
	c_closest = pomdp.Collision(Inf, Inf, pomdp.NO_COLLISION)
	cv_closest = [NaN, NaN]
  # Check for collisions with all other agents
  # If collisions are detected then get the closest one to the agent 
  for agent in world.agents
    if agent.id == agent_self.id
      continue
    end

		(c, cv) = predict_collision(agent_self.x, agent.x, world, t)
    
    if c.ctype != pomdp.NO_COLLISION && c.d < c_closest.d
			c_closest = c
			cv_closest = cv
    end
  end

	return (pomdp.sample_adv_a(agent_self.custom[5], c_closest), cv_closest)

end


function empty_controller!(x::AbstractArray{Float64},
													 u::AbstractArray{Float64},
													 dx::AbstractArray{Float64},
													 agent_world::Pair{Agent, World},
													 t::Float64)
	return
end

function parametrize(x::AbstractArray{Float64},
                     world::World)

  (sx, sy) = sim.sp2sxsy(x[1], x[3], world.road.path)
	n = norm([sx, sy])
	sx = sx/n
	sy = sy/n

  ds = x[2]

	th = atan(sy, sx)
  s = mod(x[1], world.road.path.S[end])
  p = x[3]

	v_x = ds*sx
	v_y = ds*sy
  (x_pos, y_pos) = sim.sp2xy(s, p, world.road.path)
  
  P = [x_pos v_x;
       y_pos v_y]

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

function predict_collision(x_self::Array{Float64},
													 x_opp::Array{Float64},
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
	
	# Parametrize both agents
	(P_self, th_self) = parametrize(x_self, world)
	(P_opp, th_opp) = parametrize(x_opp, world)

	# Compute Closest approach collision
	(vc_self, vc_opp, t_c) = closest_approach(P_self, P_opp)
	(x, y) = sim.sp2xy(x_self[1], x_self[3], world.road.path)

	cv = [0.0, 0.0]
	cv[1] = vc_self[1]
	cv[2] = vc_self[2]
	rv = vc_self - vc_opp

	# Compute collision angle
	th_collision = th_self - atan(cv[2], cv[1])
	
	# Determine collision type
	collision_type = pomdp.NO_COLLISION

	if norm(rv) < 10.0 && 0.0 < t_c < 1.5 # Predicted collision criteria
		if -pi / 2 < th_collision < pi / 2
			collision_type = pomdp.HITTING
		else
			collision_type = pomdp.BEING_HIT
		end
	end

  if collision_type == pomdp.NO_COLLISION
    cv = fill(NaN, length(cv))
  end

	# Compute distance of self to collision
	d = norm(cv)


	return (pomdp.Collision(d, t_c, collision_type), cv)

end

end
