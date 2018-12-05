dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/sim") # simulation
push!(LOAD_PATH, dir_path * "/vis") # visualization
push!(LOAD_PATH, dir_path * "/adv") # adversary policies
using sim
using vis
using adv

using Serialization
using Printf

function update_info(info::vis.InfoBox, agent::Agent, world::World, t::Float64)
  (x, y, sx, sy, dx, u) = diagnostic(agent, world, t)

  dp = dx[3]
  th = atan(sy, sx)
  th_car = atan(dp, agent.x[2]) + atan(sy, sx)

  vis.update_vector_thr!(info.ds_vec, info.scaling * x, 
                         info.scaling * y, th, 
                         0.2 * agent.x[2] / 50.0)
  vis.update_vector_thr!(info.u1_vec, info.scaling * x, 
                         info.scaling * y, th, 
                         0.2 * u[1] / 10.0)
  vis.update_vector_thr!(info.u2_vec, info.scaling * x, 
                         info.scaling * y, th + pi / 2, 
                         0.2 * u[2] / 0.1)

  offset = 0.0
  vis.update_text!(info.id_text, @sprintf("Agent %d", agent.id), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.s_text, @sprintf("s= %3.1e", agent.x[1]), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.ds_text, @sprintf("ds= %3.1e", agent.x[2]), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.p_text, @sprintf("p= %3.1e", agent.x[3]), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.u1_text, @sprintf("u1= %+3.1e", u[1]),
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.u2_text, @sprintf("u2= %+3.1e", u[2]),
                   info.x, info.y - offset, 1.0)
end

# Scenario: Racer scenario
function main()
  # load the graphical context (OpenGL handle, the graphical window, etc.)
  context = vis.setup()
  vis_scaling = 0.01

  # make the road
  path = make_figure8_path()
  road_width = 20.0
  road = Road(path, road_width, 
              vis.make_road(context, path.X, path.Y, road_width))
  road.road.T = vis.scale_mat(vis_scaling, vis_scaling)

  # make the world
  world = World(road, [], vis_scaling)

	len_rd = path.S[end]

  # make the agents
  agent = Agent(1, [len_rd / 2 - 50.0; 0.0; 0], vis.make_car(context))
	agent.controller! = adv.meek_controller!

	agent2 = Agent(2, [len_rd - 50.0; 0.0; 0.0], vis.make_car(context))
	agent2.controller! = adv.agro_controller!

  push!(world.agents, agent)
  push!(world.agents, agent2)
  # ------------------------------------------------------------------------- #

  # make diagnostics render objects
  info = vis.InfoBox(context, 0.75, 0.75, vis_scaling)

  # main loop for rendering and simulation
  window = true
  h = 1e-2
  t0 = time_ns()
  oldt = (time_ns() - t0) / 1e9
  while window
    t = (time_ns() - t0) / 1e9

    to_visualize = []
    if world.road.road != nothing
      push!(to_visualize, world.road.road)
    end

    for agent in world.agents
      ## advance one frame in time
      advance!(agent.dynamics!, agent.x, Pair(agent, world), oldt, t, h)
			if agent.id == 1
      	update_info(info, agent, world, t)
			end


      ## visualize
      (x, y, sx, sy, dx, u) = diagnostic(agent, world, t)
      agent.is_braking = dx[1] * u[1] < 0

      ## get additional information
      dp = dx[3]
      th = atan(sy, sx)
      th_car = atan(dp, agent.x[2]) + atan(sy, sx)

      if agent.car != nothing
        ## update the car
        vis.car_lights!(agent.car, agent.is_braking)
        agent.car.T = (vis.scale_mat(world.vis_scaling, world.vis_scaling) * 
                       vis.translate_mat(x, y) * 
                       vis.rotate_mat(th_car - pi / 2))
        push!(to_visualize, agent.car)
      end
    end
    push!(to_visualize, info)

    window = vis.visualize(context, to_visualize)

    oldt = t
  end
end
