module fwds_tree
export SearchNode

mutable struct SearchNode
  s::Array{Float64,1}
  v::Float64
  parent::Union{SearchNode,Nothing}
  children::Union{Vector{Tuple{SearchNode,Int}},Nothing}

  SearchNode(root,parent) = new(root,0.0,parent,nothing)
end

end
