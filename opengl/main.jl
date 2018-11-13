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

function print_error_if_not_empty()
  err = glErrorMessage()
  if err != ""
    println(err)
  end
end


global const attributes = Array{GLint}(undef, 4)
function make_attributes_global(program)
  positionAttribute = glGetAttribLocation(program, "position")
  colorAttribute = glGetAttribLocation(program, "color")
  usetexAttribute = glGetAttribLocation(program, "usetex")
  texcoordAttribute = glGetAttribLocation(program, "texcoord")
  attributes[:] = [positionAttribute, colorAttribute, 
                   usetexAttribute, texcoordAttribute]
end

function main()
  # Load the Window -----------------------------------------------------------
  window_width = 1080
  window_height = round(Int, 1080 * 9 / 16)
  window = make_window(window_width, window_height)
  # ---------------------------------------------------------------------------

  # Compile the Shader Program ------------------------------------------------
  vsh = read("shader.vert", String)
  fsh = read("shader.frag", String)
  program = make_shader_program(vsh, fsh)
  glUseProgram(program)
  make_attributes_global(program)
  # ---------------------------------------------------------------------------

  # Load Font Texture ---------------------------------------------------------
  textureBuffer = glGenTexture()
  glActiveTexture(GL_TEXTURE0)

  fp = open("font/font.bin", "r")
  pixels = deserialize(fp)
  close(fp)
  (w, h) = size(pixels)
  w = div(w, 4)

  glEnable(GL_BLEND)
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
  glBindTexture(GL_TEXTURE_2D, textureBuffer)
  glBindTexture(GL_TEXTURE_2D, textureBuffer)                                   
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_FLOAT, pixels);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);          
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);          
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);            
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);            

  glUniform1i(glGetUniformLocation(program, "tex"), 0) 
  # ---------------------------------------------------------------------------


  # Make a Bunch of Test Objects ----------------------------------------------
  # triangle
  data2 = GLfloat[-1.0, 1.0,
                  -1.0, -1.0,
                  -0.1, 0.0]
  color2 = GLfloat[0.0, 0.0, 1.0,
                   1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0]
  indices2 = GLuint[0, 1, 2]
  object2 = RenderObject([RenderData(data2, 2), RenderData(color2, 3),
                          RenderData(fill(GLfloat(0), 3), 1), 
                          RenderData(fill(GLfloat(0), 2 * 3), 2)],
                         attributes, indices2, STATIC)


  # road
  th = range(0, stop=(2 * pi), length=30)
  r = 0.6
  x = fill(0.0, length(th))
  y = fill(0.0, length(th))
  for i in 1:length(th)
    x[i] = r * cos(th[i])
    y[i] = r * sin(th[i])
  end

  object3 = make_road(x, y, 0.1)


  # font square
  points4 = GLfloat[-1.0, 0.5,
                    -1.0, 1.0,
                    0.0, 1.0,
                    0.0, 0.5]
  color4 = repeat(GLfloat[1.0, 0.0, 0.0], 4)
  usetex4 = fill(GLfloat(1.0), 4)
  texcoord4 = GLfloat[0.0, 1.0,
                      0.0, 0.0,
                      1.0, 0.0,
                      1.0, 1.0]
  indices4 = GLuint[0, 1, 2, 
                    0, 2, 3]
  object4 = RenderObject([RenderData(points4, 2), RenderData(color4, 3),
                          RenderData(usetex4, 1), RenderData(texcoord4, 2)],
                         attributes, indices4, STATIC)

  # line
  color = [1.0, 0.0, 0.0,
           0.0, 0.0, 1.0]
  l1 = make_line(0.0, 0.0, 0.5, -0.5, color)
  s1 = make_text(string(time_ns()), 0.0, 0.0)

  # Main Render Loop ----------------------------------------------------------
  #glViewport(0, 0, window_width, window_height)
  while !GLFW.WindowShouldClose(window)
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    #S =  scaleM(window_height / window_width, 1, 1) * rotationM(45 / 180 * pi)
    S = scaleM(window_height/window_width, 1, 1)
    glUniformMatrix4fv(glGetUniformLocation(program, "S"), 1, GL_FALSE, S)

    render(object2)
    render(object3)
    render(object4)

    t = time_ns() / 1e9
    points = GLfloat[0.0, 0.0, cos(t), sin(t)]
    update_buffer!(l1, points, attributes[1])
    render(l1)

    s1 = update_text!(s1, string(time_ns()), 0.0, 0.0)
    render(s1)


    GLFW.SwapBuffers(window)
    GLFW.PollEvents()
  end
  # ---------------------------------------------------------------------------

  # Cleanup -------------------------------------------------------------------
  GLFW.DestroyWindow(window)
  # ---------------------------------------------------------------------------

  return
end

main()
