function interp(X::AbstractArray, x::AbstractArray, v::AbstractArray)
  dim = Int(log2(length(X)))

  xi = fill(0.0, dim)
  for i in 1:dim
    xi[i] = (x[i] - X[1][i]) / (X[end][i] - X[1][i])
  end

  c = v # no need to copy
  for i in 1:dim
    len = 2^(dim - i)
    nc = fill(0.0, len)
    for j in 1:len
      nc[j] = c[2 * (j - 1) + 1] * (1.0 - xi[i]) + c[2 * (j - 1) + 2] * xi[i]
      #nc[j] = c[j] * (1.0 - xi[i]) + c[j + len] * xi[i]
    end
    c = nc
  end

  return c[]
end

X = [[0, 0, 0],
     [1, 0, 0],
     [0, 1, 0],
     [1, 1, 0],
     [0, 0, 1],
     [1, 0, 1],
     [0, 1, 1],
     [1, 1, 1]]
for i in 1:length(X)
  X[i][1] *= 7
  X[i][2] *= 17
  X[i][3] *= 9
end

f(x, y, z) = 2.0 * x + 3.0 * y +  5.0 * z
v = fill(0.0, length(X))
for i in 1:length(v)
  v[i] = f(X[i]...)
end


#=
X = [[0, 0],
     [1, 0],
     [0, 1],
     [1, 1]]
for i in 1:length(X)
  X[i][1] *= 17
  X[i][2] *= 7
end
f(x, y) = (x + y)
v = fill(0.0, length(X))

for i in 1:length(v)
  v[i] = f(X[i]...)
end
=#



#=
X = [[0],
     [1]]
for i in 1:length(X)
  X[i][1] *= 5
end
f(x) = 17.0 * x
v = fill(0.0, length(X))

for i in 1:length(v)
  v[i] = f(X[i]...)
end
=#

for i in 1:5
  x = rand(length(X[1]))
  println("x = $(x)")
  println("f(x) = $(f(x...))")
  println("I(x) = $(interp(X, x, v))")
end
