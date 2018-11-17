using LinearAlgebra

include("vis.jl")
include("rk4.jl")
include("sim_figure8.jl")

# Type Definitions ############################################################
struct Path
  X::Array{Float64}
  Y::Array{Float64}
  dX::Array{Float64}
  dY::Array{Float64}
  ddX::Array{Float64}
  ddY::Array{Float64}
  Sx::Array{Float64}
  Sy::Array{Float64}
  Px::Array{Float64}
  Py::Array{Float64}
  R::Array{Float64}
  S::Array{Float64}
end

struct Road
  path::Path
  width::Float64
  road::Union{Vis.RenderObject, Nothing}
end
Road(path, width) = Road(path, width, nothing)

mutable struct Agent
  id::Int
  x::Array{Float64, 1}
  dynamics!::Function
  controller!::Function
  car::Union{Vis.RenderObject, Nothing}
end
Agent(id, x) = Agent(id, x, default_dynamics!, default_controller!, nothing)
Agent(id, x, car::Vis.RenderObject) = Agent(id, x, default_dynamics!, 
                                            default_controller!, car)

struct World
  road::Road
  agents::Array{Agent, 1}
  vis_scaling::Float64
end
World(road::Road) = World(road, Agent[])

# Custom Rendering ############################################################
function update_renderer(agent::Agent, world::World)
  if agent.car == nothing
    return
  end

  (x, y) = sp2xy(agent.x[1], agent.x[3], world.road.path)
  (sx, sy) = sp2sxsy(agent.x[1], agent.x[3], world.road.path)
  th = atan(sy, sx) - pi / 2

  agent.car.T = (Vis.scale_mat(world.vis_scaling, world.vis_scaling) * 
                 Vis.translate_mat(x, y) * Vis.rotate_mat(th))
  return
end

# Dynamics and Control ########################################################
function default_controller!(u::AbstractArray{Float64}, 
                             x::AbstractArray{Float64}, 
                             dx::AbstractArray{Float64}, 
                             agent_world::Pair{Agent, World}, t::Float64)
  u[1] = 0 # no braking nor accelerating
  u[2] = -dx[3] # counteract lateral movement

  return
end

function default_dynamics!(dx, x, agent_world, t)
  agent = agent_world.first
  world = agent_world.second

  # extract states
  s = x[1]
  ds = x[2]
  p = x[3]

  # get local interpolated path curvature
  path = world.road.path

  s = rem(s, path.S[end])
  idx = binary_search(path.S, s)
  R = path.R[idx]
  R1 = path.R[idx + 1]
  S = path.S[idx]
  S1 = path.S[idx + 1]
  r = (R1 - R) / (S1 - S) * (s - S) + R

  # compute rates
  ds *= isnan(r / (r - p)) ? 1 : r / (r - p)
  dds = 0.0
  dp = isnan(ds^2 / r) ? 0.0 : 1e-2 * -ds^2 / r

  dx[1] = ds
  dx[2] = dds
  dx[3] = dp

  # controller acts
  u = fill(0.0, 2)
  agent.controller!(u, x, dx, agent_world, t)

  dx[1] += u[1]
  dx[3] += u[2]

  # done
  return
end

# Utility Functions ###########################################################
function sp2xy(s::Float64, p::Float64, path)
  s = rem(s, path.S[end])
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
  s = rem(s, path.S[end])
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
  b = length(sorted_x)
  if el >= sorted_x[b]
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

  return m
end

# Main Routine ################################################################
function main()
  # load the graphical context (OpenGL handle, the graphical window, etc.)
  context = Vis.setup()

  vis_scaling = 0.01

  # make the road
  path = make_figure8_path()
  road_width = 10.0
  road = Road(path, road_width, 
              Vis.make_road(context, path.X, path.Y, road_width))
  road.road.T = Vis.scale_mat(vis_scaling, vis_scaling)

  # make the agents
  agent1 = Agent(1, [0.0; 60 / 3.6; 0], Vis.make_car(context))
  Vis.car_lights!(agent1.car, false)

  agent2 = Agent(1, [0.0; 60 / 3.6; 3], Vis.make_car(context, [1.0, 1.0, 0.0]))
  Vis.car_lights!(agent2.car, false)

  agent3 = Agent(1, [0.0; 60 / 3.6; -3], Vis.make_car(context, [1.0, 0.0, 1.0]))
  Vis.car_lights!(agent3.car, false)

  # make the world
  world = World(road, [agent1, agent2, agent3], vis_scaling)

  window = true
  h = 1e-2
  t0 = time_ns()
  oldt = (time_ns() - t0) / 1e9
  while window
    t = (time_ns() - t0) / 1e9

    to_visualize = Vis.RenderObject[]
    if world.road.road != nothing
      push!(to_visualize, world.road.road)
    end

    for agent in world.agents
      rk4!(agent.dynamics!, agent.x, Pair(agent, world), oldt, t, h)
      update_renderer(agent, world)
      if agent.car != nothing
        push!(to_visualize, agent.car)
      end
    end

    window = Vis.visualize(context, to_visualize)

    oldt = t
  end
end

main()
