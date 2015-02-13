main.pdf: main.tex Makefile
	rm -f main.aux main.bbl main.blg main.brf main.lof main.log main.lot main.out main.toc
	pdflatex -halt-on-error main.tex
	pdflatex main.tex

clean:	rm -f main.aux main.bbl main.blg main.brf main.lof main.log main.lot main.out main.toc
