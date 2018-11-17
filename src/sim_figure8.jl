function make_figure8_path()
  X = Float64[]
  Y = Float64[]
  N = 100
  R = 40

  y = collect(range(0.0, stop=R, length=N))
  x = fill(0.0, length(y))
  append!(X, x)
  append!(Y, y)

  th = range(0.0, stop=(3/2 * pi), length=(3 * N))
  y = similar(th)
  x = similar(th)
  for i in 1:length(th)
    x[i] = R * cos(th[i]) - R
    y[i] = R * sin(th[i]) + R
  end
  append!(X, x)
  append!(Y, y)

  x = collect(range(-R, stop=R, length=(2 * N)))
  y = fill(0.0, length(x))
  append!(X, x)
  append!(Y, y)

  th = range(pi / 2, stop=(-pi), length=(3 * N))
  y = similar(th)
  x = similar(th)
  for i in 1:length(th)
    x[i] = R * cos(th[i]) + R
    y[i] = R * sin(th[i]) - R
  end
  append!(X, x)
  append!(Y, y)

  y = collect(range(-R, stop=0.0, length=N))
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
