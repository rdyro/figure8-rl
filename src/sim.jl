include("vis.jl")

function main()
  context = Vis.setup()

  roadX = Float64[]
  roadY = Float64[]
  N = 100

  y = collect(range(0.0, stop=0.5, length=N))
  x = fill(0.0, length(y))
  append!(roadX, x)
  append!(roadY, y)

  th = range(0.0, stop=(3/2 * pi), length=(3 * N))
  y = similar(th)
  x = similar(th)
  for i in 1:length(th)
    x[i] = 0.5 * cos(th[i]) - 0.5
    y[i] = 0.5 * sin(th[i]) + 0.5
  end
  append!(roadX, x)
  append!(roadY, y)

  x = collect(range(-0.5, stop=0.5, length=(2 * N)))
  y = fill(0.0, length(x))
  append!(roadX, x)
  append!(roadY, y)

  th = range(pi / 2, stop=(-pi), length=(3 * N))
  y = similar(th)
  x = similar(th)
  for i in 1:length(th)
    x[i] = 0.5 * cos(th[i]) + 0.5
    y[i] = 0.5 * sin(th[i]) - 0.5
  end
  append!(roadX, x)
  append!(roadY, y)

  y = collect(range(-0.5, stop=0.0, length=N))
  x = fill(0.0, length(y))
  append!(roadX, x)
  append!(roadY, y)

  remove_duplicates(roadX, roadY)

  road = Vis.make_road(roadX, roadY, 0.1)
  car = Vis.make_car()
  Vis.car_lights!(car)

  window = true
  i = 1
  frame = 0
  while window
    dx = roadX[i + 1] - roadX[i]
    dy = roadY[i + 1] - roadY[i]
    th = atan(dy, dx) - pi / 2

    car.T = (Vis.translate_mat(roadX[i], roadY[i]) * 
             Vis.rotate_mat(th) * 
             Vis.scale_mat(0.3, 0.3))
    window = Vis.visualize(context, [road, car])
    #println(time_ns())
    #if mod(frame, 3) == 0
    i += 1
    #end
    if i == length(roadX) - 1
      i = 1
    end

    frame += 1
  end
end

function remove_duplicates(x::Array{Float64}, y::Array{Float64})
  @assert length(x) == length(y)
  len = length(x)
  i = 1
  while i < len
    if x[i] == x[i + 1] && y[i] == y[i + 1]
      println("Removing a duplicate")
      deleteat!(x, i)
      deleteat!(y, i)

      len = length(x)
    end
    i += 1
  end
end

main()
