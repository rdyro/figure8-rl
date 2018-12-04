module sim

dir_path = @__DIR__
include_dependency(dir_path * "/../vis/vis.jl")

using LinearAlgebra
push!(LOAD_PATH, dir_path * "/../vis")
using vis

include("rk4.jl")
include("sim_figure8.jl")
include("sim_types.jl")

export Road, Agent, World
export advance!, make_figure8_path, diagnostic 

# Custom Rendering ############################################################
function diagnostic(agent::Agent, world::World, t::Float64=0.0)
  (x, y) = sp2xy(agent.x[1], agent.x[3], world.road.path)
  (sx, sy) = sp2sxsy(agent.x[1], agent.x[3], world.road.path)

  dx = fill(0.0, 3)
  u = fill(0.0, 2)
  agent.dynamics!(dx, agent.x, Pair(agent, world), t)
  agent.controller!(u, agent.x, dx, Pair(agent, world), t)

  return (x, y, sx, sy, dx, u)
end

# Dynamics and Control ########################################################
function default_controller!(u::AbstractArray{Float64}, 
                             x::AbstractArray{Float64}, 
                             dx::AbstractArray{Float64}, 
                             agent_world::Pair{Agent, World}, t::Float64)
  u[1] = -0.1 * (x[2] - 120 / 3.6)
  u[2] = -0.01 * x[3] / dx[1] # counteract lateral movement

  return
end

function default_dynamics!(dx::AbstractArray{Float64}, 
                           x::AbstractArray{Float64}, 
                           agent_world::Pair{Agent, World}, t::Float64)
  agent = agent_world.first
  world = agent_world.second

  # extract states
  s = x[1]
  ds = x[2]
  p = x[3]

  # get local interpolated path curvature
  path = world.road.path

  s = mod(s, path.S[end])
  idx = binary_search(path.S, s)
  if idx + 1 > length(path.R)
    display(path.S[end])
    display(s)
  end
  R = path.R[idx]
  R1 = path.R[idx + 1]
  S = path.S[idx]
  S1 = path.S[idx + 1]
  r = (R1 - R) / (S1 - S) * (s - S) + R

  # compute rates
  ds *= isnan(r / (r - p)) ? 1 : r / (r - p)
  dds = 0.0
  dds += abs(p) > world.road.width / 2 ? -1e1 * ds : 0.0
  dp = isnan(ds^2 / r) ? 0.0 : -2e-1 * ds^2 / r

  dx[1] = ds
  dx[2] = dds
  dx[3] = dp

  # controller acts
  u = fill(0.0, 2)
  agent.controller!(u, x, dx, agent_world, t)

  dx[2] += u[1]
  dx[3] += isnan(u[2]) || isinf(u[2]) ? 0.0 : ds * u[2]

  # done
  return
end

# Utility Functions ###########################################################
function sp2xy(s::Float64, p::Float64, path)
  s = mod(s, path.S[end])
  idx = binary_search(path.S, s)

  X = path.X[idx]
  X1 = path.X[idx + 1]

  Y = path.Y[idx]
  Y1 = path.Y[idx + 1]

  S = path.S[idx]
  S1 = path.S[idx + 1]

  Px = path.Px[idx]
  Px1 = path.Px[idx + 1]

  Py = path.Py[idx]
  Py1 = path.Py[idx + 1]

  x = (X1 - X) / (S1 - S) * (s - S) + X
  y = (Y1 - Y) / (S1 - S) * (s - S) + Y

  x += ((Px1 - Px) / (S1 - S) * (s - S) + Px) * p
  y += ((Py1 - Py) / (S1 - S) * (s - S) + Py) * p

  return (x, y)
end

function sp2sxsy(s::Float64, p::Float64, path)
  s = mod(s, path.S[end])
  idx = binary_search(path.S, s)

  dX = path.dX[idx]
  dX1 = path.dX[idx + 1]

  dY = path.dY[idx]
  dY1 = path.dY[idx + 1]

  S = path.S[idx]
  S1 = path.S[idx + 1]

  sx = (dX1 - dX) / (S1 - S) * (s - S) + dX
  sy = (dY1 - dY) / (S1 - S) * (s - S) + dY

  return (sx, sy)
end

function binary_search(sorted_x::Array{T}, el::T) where T
  a = 1
  len = length(sorted_x)
  b = len
  if el > sorted_x[b]
    return b
  end

  m = div(a + b, 2)
  while b - a > 1
    if sorted_x[m] > el
      b = m
    else
      a = m
    end
    m = div(a + b, 2)
  end
  m = m == len ? m - 1 : m;

  return m
end

end
