# Type Definitions ############################################################
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

struct Road
  path::Path
  width::Float64
  road::Union{Vis.RenderObject, Nothing}
end
Road(path, width) = new(path, width, nothing)

mutable struct Agent
  id::Int
  x::Array{Float64, 1}
  dynamics!::Function
  controller!::Function
  car::Union{Vis.RenderObject, Nothing}
end
Agent(id, x) = new(id, x, default_dynamics!, default_controller!, nothing)
Agent(id, x, car::Vis.RenderObject) = new(id, x, default_dynamics!, 
                                            default_controller!, car)

struct World
  road::Road
  agents::Array{Agent, 1}
  vis_scaling::Float64
end
World(road::Road) = new(road, Agent[])
