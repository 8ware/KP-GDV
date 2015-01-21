OUT := presentation

.PHONY: all
all: $(OUT)

$(OUT):
	@pdflatex $(OUT).tex
	@while grep -q "Rerun to get cross-references right." $(OUT).log; \
		do pdflatex $(OUT).tex; \
	done
	@if grep -q citation $(OUT).aux; then \
		bibtex $(OUT).aux; \
		pdflatex $(OUT).tex; \
	fi

clean:
	@rm -f *-blx.bib *.aux *.bbl *.blg *.bcf *.log *.nav *.out *.run.xml *.toc *.snm *.vrb

