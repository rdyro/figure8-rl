using GLFW
using ModernGL
using Serialization
include("util.jl")
include("drawing.jl")
include("scene.jl")

#GLFW.Init()

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

global const positionAttribute = Ref{GLint}()
global const colorAttribute = Ref{GLint}()
global const usetexAttribute = Ref{GLint}()
global const texcoordAttribute = Ref{GLint}()
function main()
  # handle GLFW loading #######################################################
  window_width = 1080
  window_height = round(Int, 1080 * 9 / 16)
  window = make_window(window_width, window_height)
  #############################################################################


  # create shader program #####################################################
  vsh = read("shader.vert", String)
  fsh = read("shader.frag", String)
  program = make_shader_program(vsh, fsh)
  glUseProgram(program)
  #############################################################################


  #############################################################################
  positionAttribute[] = glGetAttribLocation(program, "position")
  colorAttribute[] = glGetAttribLocation(program, "color")
  usetexAttribute[] = glGetAttribLocation(program, "usetex")
  texcoordAttribute[] = glGetAttribLocation(program, "texcoord")
  attributes = [positionAttribute[], colorAttribute[], 
                usetexAttribute[], texcoordAttribute[]]
  #############################################################################
  #attributes = GLint[0, 0]



  #############################################################################
  data2 = GLfloat[-1.0, 1.0,
                  -1.0, -1.0,
                  -0.1, 0.0]
  #color2 = repeat(GLfloat[0.0, 0.0, 1.0], 3)
  color2 = GLfloat[0.0, 0.0, 1.0,
                   1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0]
  indices2 = GLuint[0, 1, 2]
  object2 = draw_object([draw_data(data2, 2), draw_data(color2, 3),
                         draw_data(fill(GLfloat(0), 3), 1), 
                         draw_data(fill(GLfloat(0), 2 * 3), 2)],
                        attributes, indices2)
  #############################################################################

  th = range(0, stop=(2 * pi), length=30)
  r = 0.6
  x = fill(0.0, length(th))
  y = fill(0.0, length(th))
  for i in 1:length(th)
    x[i] = r * cos(th[i])
    y[i] = r * sin(th[i])
  end
  #x = range(-1.0, stop=1.0, length=100)
  #y = sin.(pi * x)

  object3 = make_road(x, y, 0.1, attributes)


  points4 = GLfloat[0.5, 0.5,
                    0.5, 1.0,
                    1.0, 1.0,
                    1.0, 0.5]
  color4 = repeat(GLfloat[1.0, 0.0, 0.0], 4)
  usetex4 = fill(GLfloat(1.0), 4)
  texcoord4 = GLfloat[0.0, 1.0,
                      0.0, 0.0,
                      1.0, 0.0,
                      1.0, 1.0]
  indices4 = GLuint[0, 1, 2, 
                    0, 2, 3]
  object4 = draw_object([draw_data(points4, 2), draw_data(color4, 3),
                         draw_data(usetex4, 1), draw_data(texcoord4, 2)],
                        attributes, indices4)


  textureBuffer = glGenTexture()
  glActiveTexture(GL_TEXTURE0)

  #=
  pixels = GLfloat[1.0, 1.0, 1.0,
                   0.0, 0.0, 0.0,
                   1.0, 1.0, 1.0,
                   0.0, 0.0, 0.0]
  =#
  fp = open("text/ascii.bin", "r")
  pixels = deserialize(fp)
  close(fp)
  (w, h) = size(pixels)
  w = div(w, 3)
  glBindTexture(GL_TEXTURE_2D, textureBuffer)
  glBindTexture(GL_TEXTURE_2D, textureBuffer)                                   
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_FLOAT, pixels);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);          
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);          
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);            
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);            

  glUniform1i(glGetUniformLocation(program, "tex"), 0) 







  #=
  va = glGenVertexArray()
  glBindVertexArray(va)

  pb = glGenBuffer()
  data = GLfloat[-0.5, -0.5, 
  0.0, 0.0,
  0.0, 0.0,
  0.5, 0.5]
  glBindBuffer(GL_ARRAY_BUFFER, pb)
  glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW)
  glVertexAttribPointer(positionAttribute, 2, GL_FLOAT, false, 0, C_NULL)
  glEnableVertexAttribArray(positionAttribute)

  cb = glGenBuffer()
  data = repeat(GLfloat[0.0, 0.0, 0.0], 4)
  glBindBuffer(GL_ARRAY_BUFFER, cb)
  glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW)
  glVertexAttribPointer(colorAttribute, 3, GL_FLOAT, false, 0, C_NULL)
  glEnableVertexAttribArray(colorAttribute)

  glBindVertexArray(0)
  =#

  init_lines()
  draw_line(0.0, 0.0, 0.5, 0.5, repeat([0.0], 6))

  #############################################################################
  #glViewport(0, 0, window_width, window_height)
  while !GLFW.WindowShouldClose(window)
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    #S =  scaleM(window_height / window_width, 1, 1) * rotationM(45 / 180 * pi)
    S = scaleM(window_height/window_width, 1, 1)
    glUniformMatrix4fv(glGetUniformLocation(program, "S"), 1, GL_FALSE, S)

    #render(object1)
    render(object2)
    render(object3)
    render(object4)

    render_lines()


    GLFW.SwapBuffers(window)
    GLFW.PollEvents()
  end
  #############################################################################

  #############################################################################
  GLFW.DestroyWindow(window)
  #############################################################################

  return
end

function print_error()
  err = glErrorMessage()
  if err != ""
    println(err)
  end
end

main()
