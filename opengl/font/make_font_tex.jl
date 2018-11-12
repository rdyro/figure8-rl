using ModernGL
using Images
using Serialization

function main()
  # write ascii to file
  fp = open("ascii.txt", "w")
  char_per_line = 32
  text = ""
  for j in 0:255
    c = Char(j)
    text = string(text, iscntrl(c) ? '?' : c)
    if rem(j + 1, char_per_line) == 0 && j != 255
      text = string(text, '\n')
    end
  end
  write(fp, text)
  close(fp)

  # convert ascii to a bitmap with ImageMagick
  run(`convert -background transparent -font font.ttf -pointsize 32 label:@ascii.txt font.bmp`)

  # write the bmp to row major binary data for OpenGL
  img = load("font.bmp")
  (height, width) = size(img)

  pixels = fill(GLfloat(0.0), (4 * width, height))
  for i in 1:height
    for j in 1:width
      r = GLfloat(img[i, j].r)
      g = GLfloat(img[i, j].g)
      b = GLfloat(img[i, j].b)
      a = GLfloat(img[i, j].alpha)
      pixels[4 * (j - 1) + 1, i] = r
      pixels[4 * (j - 1) + 2, i] = g
      pixels[4 * (j - 1) + 3, i] = b
      pixels[4 * (j - 1) + 4, i] = a
    end
  end

  fp = open("font.bin", "w")
  serialize(fp, pixels)
  close(fp)
  return
end

main()
