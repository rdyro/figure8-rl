module adv
dir = @__DIR__

using LinearAlgebra

include(dir * "/adv_meek.jl")
include(dir * "/adv_agro.jl")

export parametrize, closest_approach

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

  (x, y) = sim.sp2xy(s, p, world.road.path)
  
  P = [x v_x a_x/2;
       y v_y a_y/2]

  return P
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

  dist = norm(v1 - v2)

  return (dist, t)
end


end
