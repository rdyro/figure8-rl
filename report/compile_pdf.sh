#!/bin/bash
doc_name="report"
if [[ -f "$doc_name"".tex" ]]; then
  pdflatex "$doc_name"".tex"
  if [ -f *.bib ]; then
    fname="$doc_name"
    bibtex "$fname"
    pdflatex "$fname"".tex"
    pdflatex "$fname"".tex"
  fi
  if [[ "$OSTYPE" == "linux-gnu" ]]; then
    xdg-open 2> /dev/null 3> /dev/null "$doc_name"".pdf"
  elif [[ "$OSTYPE" == "darwin" ]]; then
    open 2> /dev/null 3> /dev/null "$doc_name"".pdf"
  fi
fi
