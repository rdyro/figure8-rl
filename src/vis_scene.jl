using LinearAlgebra

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
    points[2 * 2 * (i - 1) + 1] = x[i] + dpx[i] * t
    points[2 * 2 * (i - 1) + 2] = y[i] + dpy[i] * t

    points[2 * 2 * (i - 1) + 3] = x[i] - dpx[i] * t
    points[2 * 2 * (i - 1) + 4] = y[i] - dpy[i] * t

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

function make_vector(context::Context, x1::Float64, y1::Float64, 
                     x2::Float64, y2::Float64,
                     color::AbstractArray{<: Real}=GLfloat[1.0, 0.0, 0.0])
  dx = x2 - x1
  dy = y2 - y1
  r = sqrt(dx^2 + dy^2)
  th = atan(dy, dx) - pi / 2

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
    points[i] += x1
    points[i+1] += y1
  end
  color = repeat(Array{GLfloat}(color), 6)

  usetex = fill(GLfloat(0), 6)
  texcoord = fill(GLfloat(0), 2 * 6)

  return RenderObject(context, [RenderData(points, 2, GL_DYNAMIC_DRAW), 
                                RenderData(color, 3, GL_STATIC_DRAW), 
                                RenderData(usetex, 1, GL_STATIC_DRAW), 
                                RenderData(texcoord, 2, GL_STATIC_DRAW)], 6)
end

function update_vector!(context::Context, v::RenderObject, 
                        x1::Float64, y1::Float64, x2::Float64, y2::Float64,
                        color::AbstractArray{<: Real}=GLfloat[1.0, 0.0, 0.0])
  dx = x2 - x1
  dy = y2 - y1
  r = sqrt(dx^2 + dy^2)
  th = atan(dy, dx) - pi / 2

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
    points[i] += x1
    points[i+1] += y1
  end

  update_buffer!(v, points, context.attributes[1])
end
