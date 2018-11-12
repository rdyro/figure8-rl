function main()

  fp = open("ascii.txt", "w")

  d = 32

  global text = ""

  for j in 0:255
    c = Char(j)
    text = string(text, iscntrl(c) ? '?' : c)
    if rem(j + 1, d) == 0 && j != 255
      text = string(text, '\n')
    end
  end

  write(fp, text)
  close(fp)
  run(`convert -font Courier -pointsize 30 label:@ascii.txt ascii.bmp`)
  return
end

main()
