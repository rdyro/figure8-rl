using ModernGL

# Rendering Abstraction #######################################################
mutable struct RenderData
  data::AbstractArray{GLfloat}
  dim::Integer
  usage::GLenum
end

struct RenderBuffer
  id::GLuint
  size::Int
  usage::GLenum
end

mutable struct RenderObject
  vertex_array::GLuint
  render_buffers::AbstractArray{RenderBuffer}
  attributes::AbstractArray{GLint}
  elnb::Int
  idx_buffer::Union{RenderBuffer, Nothing}
end

function RenderObject(render_data::AbstractArray{RenderData},
                      attributes::AbstractArray{GLint},
                      idxXORelnb::Union{AbstractArray{GLuint}, Int})
  # save the configuration in the vertex array
  vertex_array = glGenVertexArray()
  glBindVertexArray(vertex_array)

  # allocate the buffers, buffer data and set attributes
  render_buffers = Array{RenderBuffer, 1}(undef, length(render_data))
  for i in 1:length(render_data)
    render_buffers[i] = RenderBuffer(glGenBuffer(), 
                                     sizeof(render_data[i].data), 
                                     render_data[i].usage)
    glBindBuffer(GL_ARRAY_BUFFER, render_buffers[i].id)
    # usage is GL_STATIC_DRAW or GL_DYNAMIC_DRAW
    glBufferData(GL_ARRAY_BUFFER, render_buffers[i].size, 
                 render_data[i].data, render_buffers[i].usage)
    glVertexAttribPointer(attributes[i], render_data[i].dim, GL_FLOAT, false, 
                          0, C_NULL)
    glEnableVertexAttribArray(attributes[i])
  end

  # allocate the index buffer for drawing normal objects
  idx_buffer = nothing
  elnb = 0
  if typeof(idxXORelnb) == Int
    idx_buffer = nothing
    elnb = idxXORelnb
  else
    idx = idxXORelnb
    idx_buffer = RenderBuffer(glGenBuffer(), sizeof(idx), GL_DYNAMIC_DRAW)
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, idx_buffer.id)
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(idx), idx, idx_buffer.usage)
    elnb = length(idx)
  end

  # finish saving the configuration
  glBindVertexArray(0)

  return RenderObject(vertex_array, render_buffers, attributes, elnb, 
                      idx_buffer)
end

function render(obj::RenderObject)
  glBindVertexArray(obj.vertex_array)
  if obj.idx_buffer != nothing
    glDrawElements(GL_TRIANGLES, obj.elnb, GL_UNSIGNED_INT, Ptr{Nothing}(0))
  else
    glDrawArrays(GL_LINES, 0, obj.elnb)
  end
  glBindVertexArray(0)
end


# Updating Data ###############################################################
function update_buffer!(obj::RenderObject, data::Array{GLfloat}, 
                        attribute::GLint)
  attr_idx = findfirst(x -> x == attribute, obj.attributes)
  if sizeof(data) <= obj.render_buffers[attr_idx].size
    glBindBuffer(GL_ARRAY_BUFFER, obj.render_buffers[attr_idx].id)
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(data), data)
  else
    println("RESIZING BUFFER")

    id = obj.render_buffers[attr_idx].id
    usage = obj.render_buffers[attr_idx].usage
    size = sizeof(data)
    new_buffer = RenderBuffer(id, size, usage)
    obj.render_buffers[attr_idx] = new_buffer

    glBindBuffer(GL_ARRAY_BUFFER, obj.render_buffers[attr_idx].id)
    glBufferData(GL_ARRAY_BUFFER, obj.render_buffers[attr_idx].size, data, 
                 obj.render_buffers[attr_idx].usage)
  end
end

function update_idx!(obj::RenderObject, 
                     idxXORelnb::Union{Array{GLuint}, Int})
  @assert ((obj.idx_buffer == nothing && typeof(idxXORelnb) == Int) ||
           (obj.idx_buffer != nothing && 
            typeof(idxXORelnb) == Array{GLuint, 1}))
  if obj.idx_buffer != nothing
    idx = idxXORelnb
    if sizeof(idx) <= obj.idx_buffer.size
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, obj.idx_buffer.id)
      glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(idx), idx)
    else
      println("RESIZING IDX")

      id = obj.idx_buffer.id
      usage = obj.idx_buffer.usage
      size = sizeof(idx)
      new_buffer = RenderBuffer(id, size, usage)
      obj.idx_buffer = new_buffer

      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, obj.idx_buffer.id)
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, obj.idx_buffer.size, idx, 
                   GL_DYNAMIC_DRAW)
    end
    obj.elnb = length(idx)
  else
    obj.elnb = idxXORelnb
  end
end

# Matrices ####################################################################
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

# Line Drawing ################################################################
function make_line(x1::Float64, y1::Float64, x2::Float64, y2::Float64,
                   color::AbstractArray{Float64})
  @assert length(color) == 3

  position = RenderData(GLfloat[x1, y1, x2, y2], 2, GL_DYNAMIC_DRAW)
  color = RenderData(Array{GLfloat}(repeat(color, 2)), 3, GL_STATIC_DRAW)
  usetex = RenderData(fill(GLfloat(0), 2), 1, GL_STATIC_DRAW)
  texcoord = RenderData(fill(GLfloat(0), 4), 2, GL_DYNAMIC_DRAW)
  elnb = 4
  return RenderObject([position, color, usetex, texcoord], attributes, elnb)
end

# Text Drawing ################################################################
function _make_letter(c::Char, x::Float64, y::Float64, dx::Float64, 
                      dy::Float64)
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

function _make_text_data(text::AbstractString, x::Float64, y::Float64, 
                         scale::Float64)
  offset = 0

  dx = 0.05 * scale
  dy = 0.1 * scale
  width = length(text) * dx

  P = GLfloat[]
  C = GLfloat[]
  T = GLfloat[]
  U = GLfloat[]
  I = GLuint[]

  len = (length(text) - 1) / 2
  k = 0
  for c in text
    (p, c, u, t, i) = _make_letter(c, x - (len - k) * dx, y, dx, dy)
    append!(P, p)
    append!(C, c)
    append!(U, u)
    append!(T, t)
    append!(I, i .+ offset)

    offset += 4
    k += 1
  end

  return (P, C, U, T, I)
end

function make_text(text::AbstractString, x::Float64, y::Float64, 
                   scale::Float64=1.0)
  (P, C, U, T, I) = _make_text_data(text, x, y, scale)
  return RenderObject([RenderData(P, 2, GL_DYNAMIC_DRAW), 
                       RenderData(C, 3, GL_STATIC_DRAW), 
                       RenderData(U, 1, GL_STATIC_DRAW), 
                       RenderData(T, 2, GL_DYNAMIC_DRAW)], 
                      attributes, I)
end

function update_text!(obj::RenderObject, text::AbstractString, x::Float64, 
                      y::Float64, scale::Float64=1.0)
  (P, C, U, T, I) = _make_text_data(text, x, y, scale)
  if length(I) > div(obj.idx_buffer.size, 4)
    update_buffer!(obj, C, attributes[2])
    update_buffer!(obj, U, attributes[3])
  end
  update_buffer!(obj, P, attributes[1])
  update_buffer!(obj, T, attributes[4])
  update_idx!(obj, I)

  return
end
