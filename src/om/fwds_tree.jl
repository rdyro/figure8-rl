module fwds_tree
export StateNode

mutable struct StateNode
  s::Array{Float64,1}
  v::Float64
  parent::Union{SearchNode,Nothing}
  children::Union{Vector{Tuple{SearchNode,Int}},Nothing}

  StateNode(root,parent) = new(root,0.0,parent,nothing)
end

end
