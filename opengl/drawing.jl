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
