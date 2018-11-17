using LinearAlgebra

include("vis.jl")
include("rk4.jl")

function binary_search(sorted_x::Array{T}, el::T) where T
  a = 1
  b = length(sorted_x)
  if el >= sorted_x[b]
    return b
  end

  m = div(a + b, 2)
  while b - a > 1
    if sorted_x[m] > el
      b = m
    else
      a = m
    end
    m = div(a + b, 2)
  end

  return m
end

struct Path
  X::Array{Float64}
  Y::Array{Float64}
  dX::Array{Float64}
  dY::Array{Float64}
  ddX::Array{Float64}
  ddY::Array{Float64}
  Sx::Array{Float64}
  Sy::Array{Float64}
  Px::Array{Float64}
  Py::Array{Float64}
  R::Array{Float64}
  S::Array{Float64}
end

function main()
  context = Vis.setup()

  path = make_figure8()

  road = Vis.make_road(context, path.X, path.Y, 0.1)


  u = [0.0; 1.0; 0.0]
  car = Vis.make_car(context)
  Vis.car_lights!(car, false)

  u2 = [0.0; 1.0; 0.05]
  car2 = Vis.make_car(context, [1.0, 1.0, 0.0])
  Vis.car_lights!(car2, false)

  u3 = [0.0; 1.0; -0.05]
  car3 = Vis.make_car(context, [1.0, 0.0, 1.0])
  Vis.car_lights!(car3, false)

  window = true
  frame = 0
  while window
    rk4!(car_move!, u, path, 0, 0.01)
    rk4!(car_move!, u2, path, 0, 0.01)
    rk4!(car_move!, u3, path, 0, 0.01)

    (x, y) = sp2xy(u[1], u[3], path)
    (dx, dy) = sp2dxdy(u[1], u[3], path)
    th = atan(dy, dx) - pi / 2

    car.T = (Vis.translate_mat(x, y) * Vis.rotate_mat(th) * 
             Vis.scale_mat(0.3, 0.3))

    (x, y) = sp2xy(u2[1], u2[3], path)
    (dx, dy) = sp2dxdy(u2[1], u2[3], path)
    th = atan(dy, dx) - pi / 2

    car2.T = (Vis.translate_mat(x, y) * Vis.rotate_mat(th) * 
             Vis.scale_mat(0.3, 0.3))

    (x, y) = sp2xy(u3[1], u3[3], path)
    (dx, dy) = sp2dxdy(u3[1], u3[3], path)
    th = atan(dy, dx) - pi / 2

    car3.T = (Vis.translate_mat(x, y) * Vis.rotate_mat(th) * 
             Vis.scale_mat(0.3, 0.3))

    window = Vis.visualize(context, [road, car, car2, car3])

    frame += 1
  end
end

function sp2xy(s::Float64, p::Float64, path)
  s = rem(s, path.S[end])
  idx = binary_search(path.S, s)

  X = path.X[idx]
  X1 = path.X[idx + 1]

  Y = path.Y[idx]
  Y1 = path.Y[idx + 1]

  S = path.S[idx]
  S1 = path.S[idx + 1]

  Px = path.Px[idx]
  Px1 = path.Px[idx + 1]

  Py = path.Py[idx]
  Py1 = path.Py[idx + 1]

  x = (X1 - X) / (S1 - S) * (s - S) + X
  y = (Y1 - Y) / (S1 - S) * (s - S) + Y

  x += ((Px1 - Px) / (S1 - S) * (s - S) + Px) * p
  y += ((Py1 - Py) / (S1 - S) * (s - S) + Py) * p

  return (x, y)
end

function sp2dxdy(s::Float64, p::Float64, path)
  s = rem(s, path.S[end])
  idx = binary_search(path.S, s)

  dX = path.dX[idx]
  dX1 = path.dX[idx + 1]

  dY = path.dY[idx]
  dY1 = path.dY[idx + 1]

  S = path.S[idx]
  S1 = path.S[idx + 1]

  dx = (dX1 - dX) / (S1 - S) * (s - S) + dX
  dy = (dY1 - dY) / (S1 - S) * (s - S) + dY

  return (dx, dy)
end

function car_move!(dx, x, path, t)
  s = x[1]
  ds = x[2]
  p = x[3]

  s = rem(s, path.S[end])
  idx = binary_search(path.S, s)
  R = path.R[idx]
  R1 = path.R[idx + 1]
  S = path.S[idx]
  S1 = path.S[idx + 1]
  r = (R1 - R) / (S1 - S) * (s - S) + R

  ds *= isnan(r / (r - p)) ? 1 : r / (r - p)
  dds = 0
  dp = 0

  dx[1] = ds
  dx[2] = dds
  dx[3] = dp

  return
end

function make_figure8()
  X = Float64[]
  Y = Float64[]
  N = 100

  y = collect(range(0.0, stop=0.5, length=N))
  x = fill(0.0, length(y))
  append!(X, x)
  append!(Y, y)

  th = range(0.0, stop=(3/2 * pi), length=(3 * N))
  y = similar(th)
  x = similar(th)
  for i in 1:length(th)
    x[i] = 0.5 * cos(th[i]) - 0.5
    y[i] = 0.5 * sin(th[i]) + 0.5
  end
  append!(X, x)
  append!(Y, y)

  x = collect(range(-0.5, stop=0.5, length=(2 * N)))
  y = fill(0.0, length(x))
  append!(X, x)
  append!(Y, y)

  th = range(pi / 2, stop=(-pi), length=(3 * N))
  y = similar(th)
  x = similar(th)
  for i in 1:length(th)
    x[i] = 0.5 * cos(th[i]) + 0.5
    y[i] = 0.5 * sin(th[i]) - 0.5
  end
  append!(X, x)
  append!(Y, y)

  y = collect(range(-0.5, stop=0.0, length=N))
  x = fill(0.0, length(y))
  append!(X, x)
  append!(Y, y)

  remove_duplicates(X, Y)

  # compute the first derivative and normalized
  dX = similar(X)
  dY = similar(Y)

  for i = 1:length(X)-1
    dX[i] = X[i + 1] - X[i]
    dY[i] = Y[i + 1] - Y[i]
  end
  dX[end] = X[end] - X[end-1]
  dY[end] = Y[end] - Y[end-1]


  # second derivative
  ddX = similar(dX)
  ddY = similar(dY)
  for i = 1:length(X)-1
    ddX[i] = dX[i + 1] - dX[i]
    ddY[i] = dY[i + 1] - dY[i]
  end
  ddX[end] = X[end] - X[end-1]
  ddY[end] = Y[end] - Y[end-1]

  Sx = similar(X)
  Sy = similar(X)
  Px = similar(X)
  Py = similar(X)
  for i in 1:length(X)
    len = sqrt(dX[i]^2 + dY[i]^2)
    Sx[i] = dX[i] / len
    Sy[i] = dY[i] / len

    Px[i] = -Sy[i]
    Py[i] = Sx[i]
  end

  # path and curvature
  olds = 0.0
  S = similar(X)
  R = similar(X)
  for i = 1:length(X)
    dr = [dX[i]; dY[i]]
    ddr = [ddX[i]; ddY[i]]

    k = cross([dr; 0], [ddr; 0])[3] / norm(dr, 2)^3
    R[i] = (abs(k) < 1e-3) ? Inf : 1 / k
    S[i] = olds + norm(dr, 2)
    olds = S[i]
  end

  return Path(X, Y, dX, dY, ddX, ddY, Sx, Sy, Px, Py, R, S)
end

function remove_duplicates(x::Array{Float64}, y::Array{Float64})
  @assert length(x) == length(y)
  len = length(x)
  i = 1
  while i < len
    if x[i] == x[i + 1] && y[i] == y[i + 1]
      deleteat!(x, i)
      deleteat!(y, i)

      len = length(x)
    end
    i += 1
  end
end

main()
