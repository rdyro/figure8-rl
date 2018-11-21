function advance!(f!::Function, x::Array, p, t::Number, tn::Number, h::Number)
  n = length(x)

  mem = fill(zero(eltype(x)), n * 5)
  k1 = view(mem, 1:n)
  k2 = view(mem, (n + 1):(2 * n))
  k3 = view(mem, (2 * n + 1):(3 * n))
  k4 = view(mem, (3 * n + 1):(4 * n))
  work = view(mem, (4 * n + 1):(5 * n))

  end_loop = false
  while !end_loop && t < tn
    if tn - t < h
      h = tn - t
      end_loop = true
    end

    for i = 1:n
      work[i] = x[i]
    end
    f!(k1, work, p, t)

    for i = 1:n
      work[i] = x[i] + 0.5 * h * k1[i]
    end
    f!(k2, work, p, t + 0.5 * h)

    for i = 1:n
      work[i] = x[i] + 0.5 * h * k2[i]
    end
    f!(k3, work, p, t + 0.5 * h)

    for i = 1:n
      work[i] = x[i] + h * k3[i]
    end
    f!(k4, work, p, t + h)

    for i = 1:n
      x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
    end

    t += h
  end

  return
end
