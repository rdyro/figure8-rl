using ModernGL

mutable struct draw_data{T <: Real}
  data::AbstractArray{T}
  dim::Integer
end

mutable struct draw_object
  vertex_array::GLuint
  data::AbstractArray{draw_data{T}} where T <: Real
  buffer::AbstractArray{GLuint}
  attribute::AbstractArray{GLint}
  indices::AbstractArray{GLuint}
  ind_buffer::GLuint
end

function draw_object(data::AbstractArray{draw_data{T}} where T <: Real,
                     attribute::AbstractArray{GLint},
                     indices::AbstractArray{GLuint}) 
  vertex_array = glGenVertexArray()
  glBindVertexArray(vertex_array)

  buffer = fill(GLuint(0), length(data))
  for i in 1:length(data)
    buffer[i] = glGenBuffer()
    glBindBuffer(GL_ARRAY_BUFFER, buffer[i])
    glBufferData(GL_ARRAY_BUFFER, sizeof(data[i].data), data[i].data,
                 GL_STATIC_DRAW)
    glVertexAttribPointer(attribute[i], data[i].dim, GL_FLOAT, false, 0, 
                          C_NULL)
    glEnableVertexAttribArray(attribute[i])
  end

  ind_buffer = glGenBuffer()
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ind_buffer)
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
               GL_STATIC_DRAW)
  glBindVertexArray(0)

  return draw_object(vertex_array, data, buffer, attribute, indices,
                     ind_buffer)
end

function render(obj::draw_object)
  #glEnableVertexAttribArray(positionAttribute)
  #glBindBuffer(GL_ARRAY_BUFFER, positionBuffer)
  #glVertexAttribPointer(0, 2, GL_FLOAT, false, 0, C_NULL)
  glBindVertexArray(obj.vertex_array)
  #glDrawArrays(GL_TRIANGLES, 0, 3)
  glDrawElements(GL_TRIANGLES, length(obj.indices), GL_UNSIGNED_INT,
                 Ptr{Nothing}(0))
  glBindVertexArray(0)
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

global const _lines_points = GLfloat[]
global const _lines_colors = GLfloat[]
global const _lines_va = Ref{GLint}()
global const _lines_pb = Ref{GLint}()
global const _lines_cb = Ref{GLint}()
global const _lines_ub = Ref{GLint}()
global const _lines_tb = Ref{GLint}()
# bad idea to use hash as an identity check, but this is experimental
global const _lines_last_hash = Ref{UInt}(0)

function init_lines()
  _lines_va[] = glGenVertexArray()
  glBindVertexArray(_lines_va[])

  _lines_pb[] = glGenBuffer()
  _lines_cb[] = glGenBuffer()
  _lines_ub[] = glGenBuffer()
  _lines_tb[] = glGenBuffer()

  glBindVertexArray(0)
end

function draw_line(x1::Float64, y1::Float64, x2::Float64, y2::Float64,
                   colors::AbstractArray{Float64})
  @assert length(colors) == 6

  push!(_lines_points, x1)
  push!(_lines_points, y1)
  push!(_lines_points, x2)
  push!(_lines_points, y2)

  push!(_lines_colors, colors...)
end

function clear_lines()
  empty!(_lines_points)
  empty!(_lines_colors)
end

function render_lines()
  glBindVertexArray(_lines_va[])

  new_hash = hash(_lines_points) + 23 * hash(_lines_colors) 
  if new_hash != _lines_last_hash[]
    _lines_last_hash[] = new_hash

    glBindBuffer(GL_ARRAY_BUFFER, _lines_pb[])
    glBufferData(GL_ARRAY_BUFFER, sizeof(_lines_points), _lines_points, 
                 GL_STATIC_DRAW)
    glVertexAttribPointer(positionAttribute[], 2, GL_FLOAT, false, 0, C_NULL)
    glEnableVertexAttribArray(positionAttribute[])

    glBindBuffer(GL_ARRAY_BUFFER, _lines_cb[])
    glBufferData(GL_ARRAY_BUFFER, sizeof(_lines_colors), _lines_colors, 
                 GL_STATIC_DRAW)
    glVertexAttribPointer(colorAttribute[], 3, GL_FLOAT, false, 0, C_NULL)
    glEnableVertexAttribArray(colorAttribute[])




    usetex = fill(GLfloat(0), div(length(_lines_points), 2))
    glBindBuffer(GL_ARRAY_BUFFER, _lines_ub[])
    glBufferData(GL_ARRAY_BUFFER, sizeof(usetex), usetex, GL_STATIC_DRAW)
    glVertexAttribPointer(usetexAttribute[], 1, GL_FLOAT, false, 0, C_NULL)
    glEnableVertexAttribArray(usetexAttribute[])

    texcoord = fill(GLfloat(0), length(_lines_points))
    glBindBuffer(GL_ARRAY_BUFFER, _lines_tb[])
    glBufferData(GL_ARRAY_BUFFER, sizeof(texcoord), texcoord, GL_STATIC_DRAW)
    glVertexAttribPointer(texcoordAttribute[], 2, GL_FLOAT, false, 0, C_NULL)
    glEnableVertexAttribArray(texcoordAttribute[])
  end

  glDrawArrays(GL_LINES, 0, div(length(_lines_points), 2))
  glBindVertexArray(0)
end
