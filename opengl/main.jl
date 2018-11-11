using GLFW
using ModernGL
include("util.jl")

#GLFW.Init()

function register_callbacks()
  # register window events callback
  function cursor_pos_callback(_, x, y)
    println(x)
    println(y)
  end
  function mouse_button_callback(_, button, pressed, modifier)
    println(button)
  end
  #GLFW.SetCursorPosCallback(window, cursor_pos_callback)
  #GLFW.SetMouseButtonCallback(window, mouse_button_callback)
end

function make_window(width, height)
  # Create a window and its OpenGL context
  GLFW.WindowHint(GLFW.SAMPLES, 4)
  GLFW.WindowHint(GLFW.CONTEXT_VERSION_MAJOR, 3)
  GLFW.WindowHint(GLFW.CONTEXT_VERSION_MINOR, 3)
  GLFW.WindowHint(GLFW.OPENGL_FORWARD_COMPAT, Cint(1)) # true, required by OS X
  GLFW.WindowHint(GLFW.OPENGL_PROFILE, GLFW.OPENGL_CORE_PROFILE)
  window = GLFW.CreateWindow(width, height, "window.jl")

  # Make the window's context current
  GLFW.MakeContextCurrent(window)
  return window
end

function make_shader_program(vsh::String, fsh::String)
  createcontextinfo()
  version_str = get_glsl_version_string()

  vertexShader = createShader(version_str * vsh, GL_VERTEX_SHADER)
  fragmentShader = createShader(version_str * fsh, GL_FRAGMENT_SHADER)
  program = createShaderProgram(vertexShader, fragmentShader)

  return program
end

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

function main()
  # handle GLFW loading #######################################################
  window_width = 1080
  window_height = round(Int, 1080 * 9 / 16)
  window = make_window(window_width, window_height)
  register_callbacks()
  #############################################################################


  # create shader program #####################################################
  vsh = read("shader.vert", String)
  fsh = read("shader.frag", String)
  program = make_shader_program(vsh, fsh)
  glUseProgram(program)
  #############################################################################


  #############################################################################
  positionAttribute = glGetAttribLocation(program, "position")
  colorAttribute = glGetAttribLocation(program, "color")
  texcoordAttribute = glGetAttribLocation(program, "texcoord")
  use_textureAttribute = glGetAttribLocation(program, "use_texture")
  attributes = [positionAttribute, colorAttribute, texcoordAttribute,
                use_textureAttribute]
  #############################################################################


  #############################################################################
  textureBuffer = glGenTexture()
  glActiveTexture(GL_TEXTURE0)

  function interleave(red::AbstractArray{T}, green::AbstractArray{T}, blue::AbstractArray{T}) where T <: Number
    @assert length(red) == length(green) == length(blue)
    p = fill(zero(T), 3 * length(red))
    for i in 1:length(red)
      p[3 * i - 2] = red[i]
      p[3 * i - 1] = green[i]
      p[3 * i - 0] = blue[i]
    end
    return p
  end

  width = 1000
  height = 1000

  red = fill(zero(GLfloat), width, height)
  green = fill(zero(GLfloat), width, height)
  blue = fill(zero(GLfloat), width, height)

  t = float(time_ns())
  for i in 1:width
    for j in 1:height
      red[i, j] = 2.0 * pi * i / width + t
      green[i, j] = j * i
      blue[i, j] = i / j
    end
  end
  red = sin.(red)
  red ./= findmax(red)[1]
  green ./= findmax(green)[1]
  blue ./= findmax(blue)[1]

  pixels = interleave(red, green, blue)

  glBindTexture(GL_TEXTURE_2D, textureBuffer)
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_FLOAT, pixels);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  glUniform1i(glGetUniformLocation(program, "tex"), 0)
  #############################################################################


  #############################################################################
  data = GLfloat[-1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0]
  color = repeat(GLfloat[1.0, 0.0, 1.0], 4)
  texcoord = GLfloat[0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0]
  use_texture = repeat(GLfloat[1.0], 4)
  indices = GLuint[0, 1, 2, 0, 2, 3]
  object1 = draw_object([draw_data(data, 2), draw_data(color, 3),
                         draw_data(texcoord, 2), draw_data(use_texture, 1)],
                        attributes, indices)
  #############################################################################


  #############################################################################
  data2 = GLfloat[-1.0, 1.0,
                  -1.0, -1.0,
                  -0.1, 0.0]
  color2 = repeat(GLfloat[0.0, 0.0, 1.0], 3)
  indices2 = GLuint[0, 1, 2]
  object2 = draw_object([draw_data(data2, 2), draw_data(color2, 3),
                         draw_data(fill(GLfloat(0), 2 * 3), 2),
                         draw_data(fill(GLfloat(0), 3), 1)], attributes,
                        indices2)
  #############################################################################


  #############################################################################
  while !GLFW.WindowShouldClose(window)
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    begin
      t = float(time_ns()) / 1e8
      #println("NEW TIME FRAME ###############################################")
      begin
        for i in 1:width
          val = sin(2.0 * pi * i / width + t) + 1.0
          #for j in bounds[n]:(bounds[n+1] - 1)
          for j in 1:height
            red[i, j] = val / 2.0
            green[i, j] = j * i / (width * height)
            blue[i, j] = i / j / (width)
          end
        end
        #@time red ./= findmax(red)[1]
        #@time green ./= findmax(green)[1]
        #@time blue ./= findmax(blue)[1]

        pixels = interleave(red, green, blue)

        S =  scaleM(window_height / window_width, 1, 1) * rotationM(45 / 180 * pi)
        #S = scaleM(window_height/window_width, 1, 1)
        glUniformMatrix4fv(glGetUniformLocation(program, "S"), 1, GL_FALSE, S)
        glBindTexture(GL_TEXTURE_2D, textureBuffer)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_FLOAT, pixels);
      end
    end

    render(object1)
    render(object2)

    GLFW.SwapBuffers(window)
    GLFW.PollEvents()
  end
  #############################################################################

  #############################################################################
  GLFW.DestroyWindow(window)
  #############################################################################
end

main()
