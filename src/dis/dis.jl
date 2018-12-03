module dis

struct Discretization
  pt::AbstractArray{Int, 1}      # points per
  mi::AbstractArray{Float64, 1}  # minimum
  mx::AbstractArray{Float64, 1}  # maximum

  dim::Int                       # dimension
  thr::AbstractArray{Int, 1}     # threshold
  rng::AbstractArray{Float64, 1} # range
end

function Discretization(pt, mi, mx)
  @assert length(pt) == length(mi) == length(mx)
  dim = length(pt)
  for i in 1:dim
    @assert pt[i] > 1 
    @assert mx[i] > mi[i]
  end

  dim = length(pt)
  thr = circshift(cumprod(pt), 1)
  thr[1] = 1
  rng = (mx - mi)

  Discretization(pt, mi, mx, dim, thr, rng)
end

# clamp state to within the discretization region
function clamp_x!(d::Discretization, x::AbstractArray{Float64, 1})
  @assert d.dim == length(x)
  for i in 1:d.dim
    if x[i] > d.mx[i]
      x[i] = d.mx[i]
    elseif x[i] < d.mi[i]
      x[i] = d.mi[i]
    end
  end
  return x
end

function clamp_x(d::Discretization, x::AbstractArray{Float64, 1})
  nx = copy(x)
  clamp_x!(nx)
  return nx
end

# linear state and discrete state vector
function ls2xd!(d::Discretization, xd::AbstractArray{Int, 1}, ls::Int)
  @assert d.dim == length(xd)
  for i in d.dim:-1:1
    xd[i] = div(ls, d.thr[i])
    ls = rem(ls, d.thr[i])
  end
  return xd
end

function ls2xd(d::Discretization, ls::Int)
  xd = fill(0, d.dim)
  ls2xd!(d, xd, ls)
  return xd
end

function xd2ls(d::Discretization, xd::AbstractArray{Int, 1})
  ls = 0
  for i in 1:d.dim
    @assert xd[i] < d.pt[i] && xd[i] >= 0
    ls += xd[i] * d.thr[i]
  end
  return ls
end

# discrete state vector and continuous state vector
function x2xd!(d::Discretization, xd::AbstractArray{Int, 1},
               x::AbstractArray{Float64, 1})
  @assert d.dim == length(xd) == length(x)
  for i in 1:d.dim
    @assert x[i] >= d.mi[i]
    @assert x[i] <= d.mx[i]
    xd[i] = round(Int, (d.pt[i] - 1) * (x[i] - d.mi[i]) / (d.rng[i]))
  end
  return xd
end

function x2xd(d::Discretization, x::AbstractArray{Float64, 1})
  @assert d.dim == length(x)
  xd = fill(0, d.dim)
  x2xd!(d,  xd, x)
  return xd
end

function xd2x!(d::Discretization, x::AbstractArray{Float64, 1},
               xd::AbstractArray{Int, 1})
  @assert d.dim == length(x) == length(xd)
  for i in 1:d.dim
    @assert xd[i] < d.pt[i] 
    @assert xd[i] >= 0
    x[i] = (xd[i] / (d.pt[i] - 1)) * d.rng[i] + d.mi[i]
  end
  return x
end

function xd2x(d::Discretization, xd::AbstractArray{Int, 1})
  @assert d.dim == length(xd)
  x = fill(0.0, d.dim)
  xd2x!(d, x, xd)
  return x
end

# continuous state vector and linear state
function ls2x!(d::Discretization, x::AbstractArray{Float64, 1}, ls::Int)
  xd = ls2xd(d, ls)
  xd2x!(d, x, xd)
  return x
end

function ls2x(d::Discretization, ls::Int)
  x = fill(0.0, d.dim)
  ls2x!(d, x, ls)
  return x
end

function x2ls(d::Discretization, x::AbstractArray{Float64, 1})
  xd = x2xd(d, x)
  ls = xd2ls(d, xd)
  return ls
end

# interpolation
function bounding_XD(d::Discretization, x::AbstractArray{Float64, 1})
  @assert length(x) == d.dim

  xd = x2xd(d, x)
  nx = xd2x(d, xd)

  XD = Array{AbstractArray{Int, 1}, 1}(undef, 2^(d.dim))
  for i in 1:length(XD)
    XD[i] = copy(xd)
  end
  #XD = [copy(xd) for i in 1:(2^(d.dim))]

  for i in 1:d.dim
    add = fill(0, d.dim)
    add[i] = 1
    use_add = 0

    if nx[i] >= x[i]
      add .*= -1
      use_add = 1
    end

    j = 0
    while j < length(XD)
      for k in 1:2^(i - 1)
        XD[j + k] += use_add * add
      end
      j += 2^(i - 1)
      use_add = use_add == 0 ? 1 : 0
    end
  end

  # account for a condition when a value is exactly at minimum
  for i in 1:d.dim
    malformed = false
    for j in 1:length(XD)
      if XD[j][i] == -1
        malformed = true
        break
      end
    end
    if malformed == true
      for j in 1:length(XD)
        XD[j][i] += 1
      end
    end
  end

  return XD
end

function bounding_X(d::Discretization, x::AbstractArray{Float64, 1})
  XD = bounding_XD(d, x)
  X = Array{AbstractArray{Float64, 1}, 1}(undef, length(XD))
  for i in 1:length(X)
    X[i] = xd2x(d, XD[i])
  end
  #X = [xd2x(xd, road) for xd in XD]
  return X
end

function interp(X::AbstractArray{AbstractArray{Float64, 1}, 1}, 
                x::AbstractArray{Float64, 1}, v::AbstractArray{Float64, 1})
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
    end
    c = nc
  end

  return c[]
end

function interp(d::Discretization, x::AbstractArray{Float64, 1},
                v::AbstractArray{Float64, 1})
  X = bounding_X(d, x)
  return interp(X, x, v)
end

end
