using LinearAlgebra
using ModernGL

include("drawing.jl")

function make_road(x::AbstractArray, y::AbstractArray, t::Float64)
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

    color[2 * 3 * (i - 1) + 1] = 0.0
    color[2 * 3 * (i - 1) + 2] = 0.0
    color[2 * 3 * (i - 1) + 3] = 0.0

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

  return RenderObject([RenderData(points, 2), RenderData(color, 3),
                       RenderData(usetex, 1), RenderData(texcoord, 2)],
                      attributes, indices, STATIC)
end
