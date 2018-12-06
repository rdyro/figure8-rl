dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/sim") # simulation
push!(LOAD_PATH, dir_path * "/vis") # visualization
push!(LOAD_PATH, dir_path * "/adv") # adversary policies
push!(LOAD_PATH, dir_path * "/dis") # discretization
push!(LOAD_PATH, dir_path * "/olm") 
push!(LOAD_PATH, dir_path * "/pomdp") 
using sim
using vis
using adv
using dis
using olm
using pomdp

using Serialization
using Printf

include("update_info.jl")

function discretize_adv(world::World)
  pts_per_acc = 3
  pts_per_ste = 3
  min_acc = -15
  max_acc = 5
  min_ste = -1e-1
  max_ste = 1e-1
  ctrl_d = Discretization([pts_per_acc, pts_per_ste],
                          [min_acc, min_ste],
                          [max_acc, max_ste])
  return ctrl_d
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

  # forward search agent ---------------------------------------------------- #
  x01 = [len_rd - 50.0; 25.0; 0.0]
  agent1 = Agent(1, copy(x01), vis.make_car(context))
	agent1.controller! = olm.controller_pofs!
  agent1.custom = [0.0, 0.0]
  b = fill(1 / length(pomdp.DRIVERS), length(pomdp.DRIVERS))
	ctrl_d = discretize_adv(world)
  # ------------------------------------------------------------------------- #

	v_track = 25
	p_track = 0.0
  x02 = [len_rd / 2 - 50.0; 25.0; 0]
  agent2 = Agent(2, copy(x02), vis.make_car(context, [1.0, 0.0, 0.0]))
	agent2.controller! = pomdp.adv_controller!
	agent2.custom = [v_track, p_track, pomdp.NOTHING, ctrl_d, pomdp.MEDIUM]


  push!(world.agents, agent1)
  push!(world.agents, agent2)


  # make diagnostics render objects
  info1 = vis.InfoBox(context, 0.75, 0.75, vis_scaling)
  info2 = vis.InfoBox(context, -0.75, -0.25, vis_scaling)

  v2 = vis.make_vector_xy(context, 0.0, 0.0, 0.0, 0.0, [1.0, 1.0, 1.0])
	
  # main loop for rendering and simulation
  window = true
  h = 1e-2
  t0 = time_ns()
  oldt = (time_ns() - t0) / 1e9

  frame = 0
  while window
    t = (time_ns() - t0) / 1e9

    to_visualize = []
    if world.road.road != nothing
      push!(to_visualize, world.road.road)
    end

    if mod(agent1.x[1], road.path.S[end]) < 150.0 && 
      mod(agent1.x[1], path.S[end]) > 60.0
      agent1.x = copy(x01)
      agent2.x = copy(x02)

      b = fill(1 / length(pomdp.DRIVERS), length(pomdp.DRIVERS))
      println("RESETTING")
    end

    for agent in world.agents
      cv = nothing
      if agent.id == 1
        global (u, ret) = olm.plan_pofs(agent1.x, b, agent1, agent2, world, pomdp.reward,
                          ctrl_d, 3)
        agent1.custom = u
        #(agent1.custom, _) = predict_collision(agent1.x, agent2.x, world)
        #println(pomdp.reward(agent1.x, u, agent1.x, agent1, world))
        #agent1.custom = u
      elseif agent.id == 2
        (o, cv) = adv.replan_adv(agent, world)
        agent.custom[3] = o

        print("Agent 1: ")
        (c, cv) = predict_collision(agent1.x, agent2.x, world)
        println(c.ctype)
        print("Agent 2: ")
        (c, cv) = predict_collision(agent2.x, agent1.x, world)
        println(c.ctype)
        println(c.d)
        b = pomdp.update_belief(b, o, c)

        #=
        if mod(frame, 30) == 0
          (c, _) = predict_collision(agent2.x, agent1.x, world)
          b = pomdp.update_belief(b, agent.custom[3], c)

          frame = 0
        end
        frame += 1
        =#
      end

      ## advance one frame in time
      dt = 0.1
      advance!(agent.dynamics!, agent.x, Pair(agent, world), 0.0, dt, h)
			if agent.id == 1
      	update_info(info1, agent, world, t)
      elseif agent.id == 2
      	update_info(info2, agent, world, t)
      end

      ## visualize
      (x, y, sx, sy, dx, u) = diagnostic(agent, world, t)
      agent.is_braking = dx[1] * u[1] < 0
			
      if agent.id == 2
        vis.update_vector_xy!(v2, vis_scaling * x, vis_scaling * y,
                        vis_scaling * cv[1], vis_scaling * cv[2])
      end

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

    push!(to_visualize, info1)
    push!(to_visualize, info2)
		push!(to_visualize, v2)

    window = vis.visualize(context, to_visualize)

    oldt = t
  end
end
