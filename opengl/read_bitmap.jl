using ModernGL
using Images
using ImageView
using Serialization

function store(name::AbstractString)
  img = load(name)
  (height, width) = size(img)


  pixels = fill(GLfloat(0.0), (3 * width, height))
  for i in 1:height
    for j in 1:width
      r = GLfloat(img[i, j].r)
      g = GLfloat(img[i, j].g)
      b = GLfloat(img[i, j].b)
      pixels[3 * (j - 1) + 1, i] = r
      pixels[3 * (j - 1) + 2, i] = g
      pixels[3 * (j - 1) + 3, i] = b
    end
  end

  fp = open(replace(name, r"(.*).bmp" => s"\1") * ".bin", "w")
  serialize(fp, pixels)
  close(fp)
end

function test(name::AbstractString)
  fp = open(name, "r")
  pixels = deserialize(fp)
  close(fp)

  (height, width) = size(pixels)
  height = div(height, 3)
  img = fill(RGB{Float32}(1, 1, 1), height, width)
  for i in 1:height
    for j in 1:width
      img[i, j] = RGB{Float32}(pixels[3 * (i - 1) + 1, j],
                               pixels[3 * (i - 1) + 2, j],
                               pixels[3 * (i - 1) + 3, j])
    end
  end
  imshow(img)
end
