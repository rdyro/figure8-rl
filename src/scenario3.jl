dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/sim")
push!(LOAD_PATH, dir_path * "/vis")
push!(LOAD_PATH, dir_path * "/mdp")
using sim
using vis
using mdp

# state space
const pts_per_s = 150
const pts_per_ds = 30
const pts_per_p = 20
const min_s = 0.0
const min_p = -7.0
const max_p = 7.0
const min_ds = 0.0
const max_ds = 50.0

# action space
const pts_per_acc = 10
const pts_per_ste = 3
const min_acc = -10
const max_acc = 10
const min_ste = -1
const max_ste = 1

# time increment
const dt = 1.0 / 60.0
const step_h = 1e-2

# map linear state to state vector
function ls2xd(lu::Int)
  pd = div(ls, pts_per_s * pts_per_ds)
  ls = rem(ls, pts_per_s * pts_per_ds)
  dsd = div(ls, pts_per_s)
  sd = rem(ls, pts_per_s)

  return [sd, dsd, pd]
end

function xd2ls(xd::AbstractArray{<: Real})
  sd = xd[1]
  dsd = xd[2]
  pd = xd[3]

  return sd + (dsd + pd * pts_per_ds) * pts_per_s
end

# map state vector to a discrete state vector
function x2xd(x::AbstractArray{<: Real}, road::Road)
  max_s = road.path.S[end]

  s = x[1]
  ds = x[2]
  p = x[3]

  sd = round(Int, pts_per_s * (s - min_s) / (max_s - min_s))
  dsd = round(Int, pts_per_ds * (ds - min_ds) / (max_ds - min_ds))
  pd = round(Int, pts_per_p * (p - min_p) / (max_p - min_p))

  return [sd, dsd, pd]
end

function xd2x(xd::AbstractArray{<: Real}, road::Road)
  max_s = road.path.S[end]

  sd = xd[1]
  dsd = xd[2]
  pd = xd[3]

  s = (sd / pts_per_s) * (max_s - min_s) + min_s
  ds = (dsd / pts_per_ds) * (max_ds - min_ds) + min_ds
  p = (pd / pts_per_p) * (max_p - min_p) + min_p

  return [s, ds, p]
end

# map linear input to vector input
function lu2ud(lu::Int)
  sted = div(lu, pts_per_acc)
  accd = rem(lu, pts_per_acc)

  return [accd, sted]
end

function ud2lu(ud::AbstractArray{<: Real})
  accd = ud[1]
  sted = ud[2]

  return accd + sted * pts_per_acc
end

# map input vector to a discrete input vector
function u2ud(u::AbstractArray{<: Real})
  acc = u[1]
  ste = u[2]

  accd = round(Int, pts_per_acc * (acc - min_acc) / (max_acc - min_acc))
  sted = round(Int, pts_per_ste * (ste - min_ste) / (max_ste - min_ste))

  return [accd, sted]
end

function ud2u(ud::AbstractArray{<: Real})
  accd = ud[1]
  sted = ud[2]

  acc = (accd / pts_per_acc) * (max_acc - min_acc) + min_acc
  ste = (sted / pts_per_ste) * (max_ste - min_ste) + min_ste

  return [acc, ste]
end

function make_MDP(world::World)
  @assert length(world.agents) == 1
  agent = world.agents[1]

  S = Dict{Int, DetState}()


  for s in 0:(pts_per_s * pts_per_ds * pts_per_p)
    a = collect(0:(pts_per_acc * pts_per_ste))
    a2r = fill(0.0, length(a))
    ns = fill(-1, length(a))

    xd = ls2xd(s)
    x = xd2x(xd, world.road)

    for i in 1:length(a)
      nx = copy(x)
      advance!(agent.dynamics!, nx, Pair(agent, world), 0.0, dt, step_h)
      nxd = x2xd(nx, world.road)

      ns[i] = xd2ls(nxd)
      a2r[i] = nxd[2]^2
    end

    S[s] = DetState(a, a2r, ns)
  end

end

# Scenario: Basic scenario template
function main()
  # load the graphical context (OpenGL handle, the graphical window, etc.)
  context = vis.setup()

  vis_scaling = 0.01

  # make the road
  path = make_figure8_path()
  road_width = 10.0
  global road = Road(path, road_width, 
              vis.make_road(context, path.X, path.Y, road_width))
  road.road.T = vis.scale_mat(vis_scaling, vis_scaling)

  display(maximum(road.path.S))

  # make the agents
  agent1 = Agent(1, [0.0; 0.0; 0], vis.make_car(context))
  vis.car_lights!(agent1.car, false)


  # make the world
  world = World(road, [agent1], vis_scaling)

  window = true
  h = 1e-2
  t0 = time_ns()
  oldt = (time_ns() - t0) / 1e9
  while window
    t = (time_ns() - t0) / 1e9

    to_visualize = vis.RenderObject[]
    if world.road.road != nothing
      push!(to_visualize, world.road.road)
    end

    for agent in world.agents
      advance!(agent.dynamics!, agent.x, Pair(agent, world), oldt, t, h)
      update_renderer(agent, world)
      if agent.car != nothing
        push!(to_visualize, agent.car)
      end
    end

    window = vis.visualize(context, to_visualize)

    oldt = t
  end
end

#main()