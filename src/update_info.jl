using Printf

function update_info(info::vis.InfoBox, agent::Agent, world::World, t::Float64, 
                     cv::Array{Float64, 1}=[NaN, NaN])
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
  vis.update_vector_xy!(info.c_vec, info.scaling * x, 
                        info.scaling * y, info.scaling * cv[1],
                        info.scaling * cv[2])

  offset = 0.0
  vis.update_text!(info.id_text, @sprintf("Agent %d", agent.id), info.x, info.y - offset, 1.0)
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

function update_info_from_tape(info::vis.InfoBox, 
                               agent_x::AbstractArray{Float64, 1}, 
                               agent_id::Int, diag_data::Tuple,
                               cv::Array{Float64, 1}=[NaN, NaN])
  (x, y, sx, sy, dx, u) = diag_data

  dp = dx[3]
  th = atan(sy, sx)
  th_car = atan(dp, agent_x[2]) + atan(sy, sx)

  vis.update_vector_thr!(info.ds_vec, info.scaling * x, 
                         info.scaling * y, th, 
                         0.2 * agent_x[2] / 50.0)
  vis.update_vector_thr!(info.u1_vec, info.scaling * x, 
                         info.scaling * y, th, 
                         0.2 * u[1] / 10.0)
  vis.update_vector_thr!(info.u2_vec, info.scaling * x, 
                         info.scaling * y, th + pi / 2, 
                         0.2 * u[2] / 0.1)
  vis.update_vector_xy!(info.c_vec, info.scaling * x, 
                        info.scaling * y, info.scaling * cv[1],
                        info.scaling * cv[2])

  offset = 0.0
  vis.update_text!(info.id_text, @sprintf("Agent %d", agent_id), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.s_text, @sprintf("s= %3.1e", agent_x[1]), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.ds_text, @sprintf("ds= %3.1e", agent_x[2]), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.p_text, @sprintf("p= %3.1e", agent_x[3]), 
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
