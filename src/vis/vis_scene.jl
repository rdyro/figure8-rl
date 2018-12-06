using LinearAlgebra

struct InfoBox
  context::Context
  x::Float64
  y::Float64
  scaling::Float64

  ds_vec::RenderObject
  u1_vec::RenderObject
  u2_vec::RenderObject
  c_vec::RenderObject

  id_text::RenderObject
  s_text::RenderObject
  ds_text::RenderObject
  p_text::RenderObject
  u1_text::RenderObject
  u2_text::RenderObject

  box::RenderObject
end

function InfoBox(context::Context, x::Float64, y::Float64, scaling::Float64)
  ds_vec = vis.make_vector_xy(context, 0.0, 0.0, 0.0, 0.0, [1.0, 0.0, 0.0])
  u1_vec = vis.make_vector_xy(context, 0.0, 0.0, 0.0, 0.0, [1.0, 1.0, 0.0])
  u2_vec = vis.make_vector_xy(context, 0.0, 0.0, 0.0, 0.0, [1.0, 1.0, 0.0])
  c_vec = vis.make_vector_xy(context, 0.0, 0.0, 0.0, 0.0, [1.0, 1.0, 1.0])

  id_text = vis.make_text(context, "test", 0.0, 0.0, 0.5)
  s_text = vis.make_text(context, "test", 0.0, 0.0, 0.5)
  ds_text = vis.make_text(context, "test", 0.0, 0.0, 0.5)
  p_text = vis.make_text(context, "test", 0.0, 0.0, 0.5)
  u1_text = vis.make_text(context, "test", 0.0, 0.0, 0.5)
  u2_text = vis.make_text(context, "test", 0.0, 0.0, 0.5)

  position = RenderData(fill(GLfloat(0.0), 2 * 8), 2, GL_DYNAMIC_DRAW)
  color = RenderData(fill(GLfloat(0.0), 3 * 8), 3, GL_STATIC_DRAW)
  usetext = RenderData(fill(GLfloat(0.0), 1 * 8), 1, GL_STATIC_DRAW)
  texcoord = RenderData(fill(GLfloat(0.0), 2 * 8), 2, GL_STATIC_DRAW)
  elnb = 8

  box = RenderObject(context, [position, color, usetext, texcoord], elnb)

  return InfoBox(context, x, y, scaling, ds_vec, u1_vec, u2_vec, c_vec,
                 id_text, s_text, ds_text, p_text, u1_text, u2_text, box)
end

function render(info::InfoBox)
  xmin = Inf
  xmax = -Inf
  ymin = Inf
  ymax = -Inf
  all_text = [info.id_text, info.s_text, info.ds_text, info.p_text, 
              info.u1_text, info.u2_text]
  for text in all_text
    (_xmin, _ymin, _xmax, _ymax) = bounding_box(text)
    xmin = min(xmin, _xmin)
    xmax = max(xmax, _xmax)
    ymin = min(ymin, _ymin)
    ymax = max(ymax, _ymax)

    render(text)
  end
  render(info.ds_vec)
  render(info.u1_vec)
  render(info.u2_vec)
  render(info.c_vec)
  position = GLfloat[xmin, ymin, 
                     xmin, ymax, 

                     xmin, ymax,
                     xmax, ymax, 

                     xmax, ymax, 
                     xmax, ymin, 

                     xmax, ymin, 
                     xmin, ymin]
  update_buffer!(info.box, position, info.context.attributes[1])
  render(info.box)
end

function make_road(context::Context, x::AbstractArray, y::AbstractArray, 
                   t::Number)
  @assert length(x) == length(y) > 1
  len = length(x)

  # figure out the gradient
  dpx = similar(x)
  dpy = similar(y)

  for i in 1:len
    upidx = i == len ? len : i + 1
    dwidx = i == 1 ? 1 : i - 1

    dx = x[upidx] - x[dwidx]
    dy = y[upidx] - y[dwidx]

    n = sqrt(dx^2 + dy^2)

    dpx[i] = -dy / n
    dpy[i] = dx / n
  end

  # allocate CPU memory for vertices objects
  points = fill(GLfloat(0), 2 * 2 * len)
  color = fill(GLfloat(0), 2 * 3 * len)
  usetex = fill(GLfloat(0), 2 * len)
  texcoord = fill(GLfloat(0), 2 * 2 * len)
  indices = fill(GLuint(0), 2 * 3 * (len - 1))

  # create points and colors
  for i in 1:len
    points[2 * 2 * (i - 1) + 1] = x[i] + dpx[i] * t / 2
    points[2 * 2 * (i - 1) + 2] = y[i] + dpy[i] * t / 2

    points[2 * 2 * (i - 1) + 3] = x[i] - dpx[i] * t / 2
    points[2 * 2 * (i - 1) + 4] = y[i] - dpy[i] * t / 2

    color[2 * 3 * (i - 1) + 1] = 0.15
    color[2 * 3 * (i - 1) + 2] = 0.15
    color[2 * 3 * (i - 1) + 3] = 0.15

    color[2 * 3 * (i - 1) + 4] = color[2 * 3 * (i - 1) + 1]
    color[2 * 3 * (i - 1) + 5] = color[2 * 3 * (i - 1) + 2]
    color[2 * 3 * (i - 1) + 6] = color[2 * 3 * (i - 1) + 3]
  end

  # select points to form triangles
  for i in 1:(len - 1)
    indices[2 * 3 * (i - 1) + 1] = 2 * (i - 1) + 1 - 1
    indices[2 * 3 * (i - 1) + 2] = 2 * (i - 1) + 2 - 1
    indices[2 * 3 * (i - 1) + 3] = 2 * (i) + 1 - 1

    indices[2 * 3 * (i - 1) + 4] = 2 * (i) + 1 - 1
    indices[2 * 3 * (i - 1) + 5] = 2 * (i) + 2 - 1
    indices[2 * 3 * (i - 1) + 6] = 2 * (i - 1) + 2 - 1
  end

  return RenderObject(context, [RenderData(points, 2, GL_STATIC_DRAW), 
                                RenderData(color, 3, GL_STATIC_DRAW),
                                RenderData(usetex, 1, GL_STATIC_DRAW), 
                                RenderData(texcoord, 2, GL_STATIC_DRAW)], 
                      indices)
end

# dodger blue
function make_car(context::Context, 
                  car_color::Array{<: Number, 1}=GLfloat[0.12, 0.56, 1.0])
  width = 1.7
  length = 4.5

  flpos = 0.6 * width / 2
  flw = 0.3 * width
  fll = 0.3 * width

  blpos = 0.6 * width / 2
  blw = 0.3 * width
  bll = 0.3 * width
  position_data = GLfloat[
                          # car
                          -width / 2, -length / 2,
                          -width / 2, length / 2,
                          width / 2, length / 2,
                          width / 2, -length / 2,
                          # left front light
                          -flpos - flw / 2, length / 2 - fll / 2,
                          -flpos - flw / 2, length / 2 + fll / 2,
                          -flpos + flw / 2, length / 2 + fll / 2,
                          -flpos + flw / 2, length / 2 - fll / 2,
                          # right front light
                          flpos - flw / 2, length / 2 - fll / 2,
                          flpos - flw / 2, length / 2 + fll / 2,
                          flpos + flw / 2, length / 2 + fll / 2,
                          flpos + flw / 2, length / 2 - fll / 2,
                          # left front light
                          -blpos - blw / 2, -length / 2 - bll / 2,
                          -blpos - blw / 2, -length / 2 + bll / 2,
                          -blpos + blw / 2, -length / 2 + bll / 2,
                          -blpos + blw / 2, -length / 2 - bll / 2,
                          # right front light
                          blpos - blw / 2, -length / 2 - bll / 2,
                          blpos - blw / 2, -length / 2 + bll / 2,
                          blpos + blw / 2, -length / 2 + bll / 2,
                          blpos + blw / 2, -length / 2 - bll / 2
                         ]
  position = RenderData(position_data, 2, GL_STATIC_DRAW)

  fl_color = GLfloat[1.0, 1.0, 1.0]
  bl_color = GLfloat[1.0, 0.0, 0.0]
  color_data = [repeat(Array{GLfloat}(car_color), 4); 
                repeat(fl_color, 8); repeat(bl_color, 8)]
  color = RenderData(color_data, 3, GL_STATIC_DRAW)

  usetex = RenderData(fill(GLfloat(0), 20), 1, GL_STATIC_DRAW)
  texcoord = RenderData(fill(GLfloat(0), 2 * 20), 2, GL_STATIC_DRAW)

  idx1 = GLuint[0, 1, 2,
                0, 2, 3]
  idx = GLuint[]
  offset = GLuint(0)
  for i in 1:5
    append!(idx, idx1 .+ offset)
    offset += 4
  end

  return RenderObject(context, [position, color, usetex, texcoord], idx)
end

function car_lights!(car::RenderObject, on::Union{Bool, Nothing}=nothing)
  @assert car.render_buffers[1].size == 4 * 4 * 2 * 5
  if on == nothing
    car.elnb = (car.elnb == 3 * 2 * 3) ? 3 * 2 * 5 : 3 * 2 * 3 # toggle
  else
    if on == true
      car.elnb = 3 * 2 * 5 # turn on
    else
      car.elnb = 3 * 2 * 3 # turn off
    end
  end
end

# Vectors #####################################################################
function make_vector_thr(context::Context, x::Float64, y::Float64, 
                         th::Float64, r::Float64,
                         color::AbstractArray{<: Real}=GLfloat[1.0, 0.0, 0.0])
  th = th - pi / 2

  arrow_r = 0.015
  points = GLfloat[0, 0,
                   0, r,
                   -arrow_r, r - arrow_r * 2,
                   0, r,
                   arrow_r, r - arrow_r * 2,
                   0, r]
  rotM = [cos(th) -sin(th);
          sin(th) cos(th)]
  for i in 1:2:length(points)
    points[i:i+1] = rotM * points[i:i+1]
    points[i] += x
    points[i+1] += y
  end
  color = repeat(Array{GLfloat}(color), 6)

  usetex = fill(GLfloat(0), 6)
  texcoord = fill(GLfloat(0), 2 * 6)

  return RenderObject(context, [RenderData(points, 2, GL_DYNAMIC_DRAW), 
                                RenderData(color, 3, GL_STATIC_DRAW), 
                                RenderData(usetex, 1, GL_STATIC_DRAW), 
                                RenderData(texcoord, 2, GL_STATIC_DRAW)], 6)
end

function make_vector_xy(context::Context, x1::Float64, y1::Float64, 
                        x2::Float64, y2::Float64,
                        color::AbstractArray{<: Real}=GLfloat[1.0, 0.0, 0.0])
  dx = x2 - x1
  dy = y2 - y1
  th = atan(dy, dx)
  r = sqrt(dx^2 + dy^2)

  return make_vector_thr(context, x1, y1, th, r, color)
end

function update_vector_thr!(v::RenderObject, 
                            x::Float64, y::Float64, th::Float64, r::Float64)
  th = th - pi / 2

  if r < 0.0
    r = -r
    th += pi
  end

  arrow_r = 0.015
  points = GLfloat[0, 0,
                   0, r,
                   -arrow_r, r - arrow_r * 2,
                   0, r,
                   arrow_r, r - arrow_r * 2,
                   0, r]
  rotM = [cos(th) -sin(th);
          sin(th) cos(th)]
  for i in 1:2:length(points)
    points[i:i+1] = rotM * points[i:i+1]
    points[i] += x
    points[i+1] += y
  end

  update_buffer!(v, points, v.context.attributes[1])
end

function update_vector_xy!(v::RenderObject, 
                           x1::Float64, y1::Float64, x2::Float64, y2::Float64)
  dx = x2 - x1
  dy = y2 - y1
  r = sqrt(dx^2 + dy^2)
  th = atan(dy, dx)

  update_vector_thr!(v, x1, y1, th, r)
end
