using ModernGL

@enum RenderType begin
  STATIC
  DYNAMIC
end

mutable struct RenderData
  data::AbstractArray{GLfloat}
  dim::Integer
end

mutable struct RenderObject
  vertex_array::GLuint
  render_data::AbstractArray{RenderData}
  render_buffers::AbstractArray{GLuint}
  attributes::AbstractArray{GLint}
  indices::Union{AbstractArray{GLuint}, Nothing}
  idx_buffer::Union{GLuint, Nothing}
  tp::RenderType
end

function RenderObject(render_data::AbstractArray{RenderData},
                      attributes::AbstractArray{GLint},
                      indices::Union{AbstractArray{GLuint}, Nothing}, 
                      tp::RenderType) 
  vertex_array = glGenVertexArray()
  glBindVertexArray(vertex_array)

  render_buffers = fill(GLuint(0), length(render_data))
  for i in 1:length(render_data)
    render_buffers[i] = glGenBuffer()
    glBindBuffer(GL_ARRAY_BUFFER, render_buffers[i])
    glBufferData(GL_ARRAY_BUFFER, sizeof(render_data[i].data), 
                 render_data[i].data, GL_STATIC_DRAW)
    glVertexAttribPointer(attributes[i], render_data[i].dim, GL_FLOAT, false, 
                          0, C_NULL)
    glEnableVertexAttribArray(attributes[i])
  end

  idx_buffer = nothing
  if indices != nothing
    idx_buffer = glGenBuffer()
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, idx_buffer)
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, 
                 GL_STATIC_DRAW)
  end
  glBindVertexArray(0)

  return RenderObject(vertex_array, render_data, render_buffers, attributes, 
                      indices, idx_buffer, tp)
end

function render(obj::RenderObject)
  glBindVertexArray(obj.vertex_array)
  if obj.indices != nothing
    glDrawElements(GL_TRIANGLES, length(obj.indices), GL_UNSIGNED_INT,
                   Ptr{Nothing}(0))
  else
    glDrawArrays(GL_LINES, 0, div(length(obj.render_data[1].data), 
                                  obj.render_data[1].dim))
  end
  glBindVertexArray(0)
end

function update_buffer!(obj::RenderObject, render_data::Array{GLfloat},
                        attribute::GLint)
  attr_idx = findfirst(x -> x == attribute, obj.attributes)
  @assert sizeof(render_data) == sizeof(obj.render_data[attr_idx].data)

  #glBindVertexArray(obj.vertex_array)
  glBindBuffer(GL_ARRAY_BUFFER, obj.render_buffers[attr_idx])
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(render_data), 
                  render_data)
  #glBindVertexArray(0)
end

function scaleM(x, y, z)
  return GLfloat[x 0 0 0;
                 0 y 0 0;
                 0 0 z 0;
                 0 0 0 1]
end


function rotationM(th)
  return GLfloat[cos(th) -sin(th) 0 0;
                 sin(th) cos(th) 0 0;
                 0 0 1 0;
                 0 0 0 1]
end

function make_line(x1::Float64, y1::Float64, x2::Float64, y2::Float64,
                   colors::AbstractArray{Float64})
  @assert length(colors) == 6

  return RenderObject([RenderData(GLfloat[x1, y1, x2, y2], 2),
                       RenderData(Array{GLfloat}(colors), 3),
                       RenderData(fill(GLfloat(0), 2), 1),
                       RenderData(fill(GLfloat(0), 4), 2)], attributes, 
                      nothing, DYNAMIC)
end

function _make_letter(c::Char, x::Float64, y::Float64, 
                      dx::Float64, dy::Float64)
  tx = rem(Int(c), 32) / 32
  ty = div(Int(c), 32) / 8

  tdx = 1 / 32
  tdy = 1 / 8

  position = GLfloat[x - dx / 2, y - dy / 2,
                     x - dx / 2, y + dy / 2,
                     x + dx / 2, y + dy / 2,
                     x + dx / 2, y - dy / 2]
  colors = fill(GLfloat(1), 3 * 4)
  usetex = fill(GLfloat(1), 4)
  texcoord = GLfloat[tx, ty + tdy,
                     tx, ty,
                     tx + tdx, ty,
                     tx + tdx, ty + tdy]
  indices = GLuint[0, 1, 2,
                   0, 2, 3]

  return (position, colors, usetex, texcoord, indices)
end

function make_text(s::AbstractString, x::Float64, y::Float64)
  offset = 0

  dx = 0.05
  dy = 0.1
  width = length(s) * dx

  P = GLfloat[]
  C = GLfloat[]
  T = GLfloat[]
  U = GLfloat[]
  I = GLuint[]

  len = (length(s) - 1) / 2
  k = 0
  for c in s
    (p, c, u, t, i) = _make_letter(c, x - (len - k) * dx, y, dx, dy)
    append!(P, p)
    append!(C, c)
    append!(U, u)
    append!(T, t)
    append!(I, i .+ offset)

    offset += 4
    k += 1
  end

  return RenderObject([RenderData(P, 2), RenderData(C, 3), RenderData(U, 1), 
                       RenderData(T, 2)], attributes, I, DYNAMIC)
end

function update_text!(obj::RenderObject, s::AbstractString, x::Float64, 
                      y::Float64)
  tdx = 1 / 32
  tdy = 1 / 8

  if div(length(obj.render_data[1].data), 
         obj.render_data[1].dim * 4) == length(s)
    T = GLfloat[]
    for c in s
      tx = rem(Int(c), 32) / 32
      ty = div(Int(c), 32) / 8
      texcoord = GLfloat[tx, ty + tdy,
                         tx, ty,
                         tx + tdx, ty,
                         tx + tdx, ty + tdy]
      append!(T, texcoord)
    end
    update_buffer!(obj, T, attributes[4])
    return obj
  else
    println("Making new text")
    return make_text(s, x, y)
  end
end
