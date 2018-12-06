module tap

dir = @__DIR__
push!(LOAD_PATH, dir * "/../sim")
push!(LOAD_PATH, dir * "/../vis")
using sim
using vis

include(dir * "/../update_info.jl")

mutable struct TapeFrame
  X::Array{Array{Float64, 1}, 1}
  DIAG::Array{Tuple, 1}
  CV::Array{Array{Float64, 1}, 1}

  TapeFrame() = new(Array{Float64, 1}[], Tuple[], Array{Float64, 1}[])
end

mutable struct Tape
  world::World
  frames::Array{TapeFrame, 1}
  info_objs::Array{Union{vis.InfoBox, Nothing}, 1}
  frame_last::Int

  function Tape(world)
    info_objs = Array{Union{vis.InfoBox, Nothing}, 1}(undef, 
                                                      length(world.agents))
    for i in 1:length(info_objs)
      info_objs[i] = nothing
    end
    new(world, TapeFrame[], info_objs, 0)
  end
end

function add_frame!(tape::Tape)
  push!(tape.frames, TapeFrame())
  tape.frame_last += 1
end

function add_to_frame!(tape::Tape, x::Array{Float64, 1}, diag::Tuple, 
                       cv::Array{Float64, 1}=[NaN, NaN])
  push!(tape.frames[tape.frame_last].X, x)
  push!(tape.frames[tape.frame_last].DIAG, diag)
  push!(tape.frames[tape.frame_last].CV, cv)
end

function visualize_frame(tape::Tape, f::Int)
  @assert f > 0 && f <= length(tape.frames)

  to_vis = []
  if tape.world.road.road != nothing
    push!(to_vis, tape.world.road.road)
  end
  i = 1
  for agent in tape.world.agents
    agent_x = tape.frames[f].X[i]
    diag = tape.frames[f].DIAG[i]
    cv = tape.frames[f].CV[i]

    if tape.info_objs[i] != nothing
      update_info_from_tape(tape.info_objs[i], agent_x, agent.id, diag, cv)
      push!(to_vis, tape.info_objs[i])
    end
    (x, y, sx, sy, dx, u) = diag
    ## get additional information
    dp = dx[3]
    th = atan(sy, sx)
    th_car = atan(dp, agent_x[2]) + atan(sy, sx)
    agent_is_braking = dx[1] * u[1] < 0
    if agent.car != nothing
      ## update the car
      vis.car_lights!(agent.car, agent_is_braking)
      agent.car.T = (vis.scale_mat(tape.world.vis_scaling, 
                                   tape.world.vis_scaling) *
                     vis.translate_mat(x, y) *
                     vis.rotate_mat(th_car - pi / 2))
      push!(to_vis, agent.car)
    end
    i += 1
  end

  return to_vis
end

end
