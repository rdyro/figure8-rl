dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/sim")
push!(LOAD_PATH, dir_path * "/vis")
push!(LOAD_PATH, dir_path * "/mdp")
push!(LOAD_PATH, dir_path * "/om")

using sim
using vis
using mdp
using fwds_tree

using Serialization

# state space
const pts_per_s = 150
const pts_per_ds = 30
const pts_per_p = 20
const min_s = 0.0
const min_p = -12.0
const max_p = 12.0
const min_ds = 0.0
const max_ds = 50.0

# action space
const pts_per_acc = 10
const pts_per_ste = 3
const min_acc = -10
const max_acc = 10
const min_ste = -1e-1
const max_ste = 1e-1

# time increment
const dt = 0.5
const step_h = 1e-1

# map linear state to state vector
function ls2xd(ls::Int)
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

  # enforce position within max_s
  x[1] = mod(x[1], max_s)

  s = x[1]
  ds = x[2]
  p = x[3]

  sd = round(Int, (pts_per_s - 1) * (s - min_s) / (max_s - min_s))
  dsd = round(Int, (pts_per_ds - 1) * (ds - min_ds) / (max_ds - min_ds))
  pd = round(Int, (pts_per_p - 1) * (p - min_p) / (max_p - min_p))

  return [sd, dsd, pd]
end

function xd2x(xd::AbstractArray{<: Real}, road::Road)
  max_s = road.path.S[end]

  sd = xd[1]
  dsd = xd[2]
  pd = xd[3]

  s = (sd / (pts_per_s - 1)) * (max_s - min_s) + min_s
  ds = (dsd / (pts_per_ds - 1)) * (max_ds - min_ds) + min_ds
  p = (pd / (pts_per_p - 1)) * (max_p - min_p) + min_p

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

  accd = round(Int, (pts_per_acc - 1) * (acc - min_acc) / (max_acc - min_acc))
  sted = round(Int, (pts_per_ste - 1) * (ste - min_ste) / (max_ste - min_ste))

  return [accd, sted]
end

function ud2u(ud::AbstractArray{<: Int})
  accd = ud[1]
  sted = ud[2]

  acc = (accd / (pts_per_acc - 1)) * (max_acc - min_acc) + min_acc
  ste = (sted / (pts_per_ste - 1)) * (max_ste - min_ste) + min_ste

  return [acc, ste]
end

function next_state!(x::AbstractArray{Float64}, lu::Int, agent_world::Pair{Agent, World})
	dt = 1.0 
	h = 1e-2
	agent.custom = lu
	advance!(agent_world.first.dynamics!, x, agent_world, 0.0, dt, h)
end

function forward_search!(node::StateNode,
												 depth::Int,
												 agent_world::Pair{Agent, World},
												 discount::Float64)
	
	if depth == 0
		return
	end

	for lu in 0:(pts_per_acc * pts_per_ste - 1)
		nx = copy(node.s)

		next_state!(nx, lu, agent_world)
		r = abs(nx[3]) > 0.7 * agent_world.second.road.width ? -1e5 : nx[2]^2

		child = StateNode(nx, node)
		child.v = r
		
		node.v += discount*r

		if node.children == nothing
			node.children = [(child, lu)]
		else
			push!(node.children,(child, lu))
		end
	end

	if node.parent != nothing
		node.parent.v += node.v
	end

	for child_a in node.children
		forward_search!(child_a[1], depth - 1, agent_world, discount)
	end

	return
end

function controller_fwds!(u::AbstractArray{Float64},
                      x::AbstractArray{Float64},
                      dx::AbstractArray{Float64},
                      agent_world::Pair{Agent, World}, t::Float64)
	# Controller for forward search
	agent = agent_world.first
	agent.controller! = controllerd!
	agent.dynamics! = dynamicsd!
	discount = 0.999

	T = StateNode(x,nothing) # Initialize Tree	
	depth = 1
	forward_search!(T,depth,agent_world,discount)
	
	best_lu = -1
	best_v = -Inf

	for child_a in T.children
		if child_a[1].v > best_v
			best_lu = child_a[2]
			best_v = child_a[1].v
		end
	end

	agent.controller! = controller_fwds!
	agent.dynamics! = sim.default_dynamics!

  ud = lu2ud(best_lu)
	
	u = ud2u(ud)

	@show(u,best_v)
	return
end

function controllerd!(u::AbstractArray{Float64},
                      x::AbstractArray{Float64},
                      dx::AbstractArray{Float64},
                      agent_world::Pair{Agent, World}, t::Float64)
  agent = agent_world.first
  world = agent_world.second

  ud = lu2ud(agent.custom)
  agent_u = ud2u(ud)

  u[1] = agent_u[1]
  u[2] = agent_u[2]

  return
end

function dynamicsd!(dx::AbstractArray{Float64},
                    x::AbstractArray{Float64},
                    agent_world::Pair{Agent, World}, t::Float64)

  agent = agent_world.first
  world = agent_world.second

  max_s = world.road.path.S[end]

  sim.default_dynamics!(dx, x, agent_world, t)

  # enforce max values
  x[1] = x[1] > max_s ? max_s : x[1]
  x[1] = x[1] < min_s ? min_s : x[1]

  x[2] = x[2] > max_ds ? max_ds : x[2]
  x[2] = x[2] < min_ds ? min_ds : x[2]

  x[3] = x[3] > max_p ? max_p : x[3]
  x[3] = x[3] < min_p ? min_p : x[3]

  return
end

# Scenario: Basic scenario template
function main()
  # load the graphical context (OpenGL handle, the graphical window, etc.)
  context = vis.setup()

  vis_scaling = 0.01

  # make the road
  path = make_figure8_path()
  road_width = 20.0
  global road = Road(path, road_width, 
                     vis.make_road(context, path.X, path.Y, road_width))
  road.road.T = vis.scale_mat(vis_scaling, vis_scaling)

  display(maximum(road.path.S))

  # make the agents
  global agent = Agent(1, [25.0; 0.0; 0], vis.make_car(context))
  vis.car_lights!(agent.car, false)

	# agent.dynamicsd!
	agent.controller! = controller_fwds!

  # make the world
  global world = World(road, [agent], vis_scaling)

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
			
			(x, y, sx, sy, dx, u) = sim.diagnostic(agent, world, t)
      agent.is_braking = dx[1] * u[1] < 0

      ## get additional information
      dp = dx[3]
      th = atan(sy, sx)
      th_car = atan(dp, agent.x[2]) + atan(sy, sx)

      if agent.car != nothing
				vis.car_lights!(agent.car, agent.is_braking)
        agent.car.T = (vis.scale_mat(world.vis_scaling, world.vis_scaling) *
                       vis.translate_mat(x, y) *
                       vis.rotate_mat(th_car - pi / 2))
        push!(to_visualize, agent.car)
      end
    end

    window = vis.visualize(context, to_visualize)

    oldt = t
  end
end

