dir = @__DIR__

push!(LOAD_PATH, dir * "/../sim")
push!(LOAD_PATH, dir * "/../vis")
push!(LOAD_PATH, dir)

using sim
using vis
using adv

include("adv_weak.jl")

function main()
	vis_scaling = 0.01

  # make the road
  path = make_figure8_path()
  road_width = 20.0
  road = Road(path, road_width)

  # make the world
  world = sim.World(road, [], vis_scaling)

	len_rd = path.S[end]

	P1 = parametrize([len_rd / 2 - 10.0, 5.0, 0.0], [5.0, 2.0, 0.1], world)
	P2 = parametrize([len_rd - 10.0, 5.0, 0.0], [5.0, 2.0, 20.0], world)
	(dist,t_closest) = closest_approach(P1, P2)

	@show(dist, t_closest)
	

	return
end
