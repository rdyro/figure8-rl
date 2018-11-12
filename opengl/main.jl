using GLFW
using ModernGL
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
  positionAttribute = glGetAttribLocation(program, "position")
  colorAttribute = glGetAttribLocation(program, "color")
  attributes = [positionAttribute, colorAttribute]
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
  object2 = draw_object([draw_data(data2, 2), draw_data(color2, 3)], 
                        attributes, indices2)
  #############################################################################

  th = range(0, stop=(2 * pi), length=1000)
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

  #############################################################################
  while !GLFW.WindowShouldClose(window)
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    #S =  scaleM(window_height / window_width, 1, 1) * rotationM(45 / 180 * pi)
    S = scaleM(window_height/window_width, 1, 1)
    glUniformMatrix4fv(glGetUniformLocation(program, "S"), 1, GL_FALSE, S)

    #render(object1)
    render(object2)
    render(object3)

    GLFW.SwapBuffers(window)
    GLFW.PollEvents()
  end
  #############################################################################

  #############################################################################
  GLFW.DestroyWindow(window)
  #############################################################################
end

main()
