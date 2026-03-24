TARGET = root
FINAL_TARGET = ctlio_jacobians
NKF = -u

all:
	nkf $(NKF) --overwrite *.tex
	uplatex $(TARGET).tex
	uplatex $(TARGET).tex

pdf:
	nkf $(NKF) --overwrite *.tex
	uplatex $(TARGET).tex
	uplatex $(TARGET).tex
	xdvipdfmx $(TARGET).dvi

pdfopt:
	nkf $(NKFOPTION) --overwrite ./*.tex
	uplatex $(TARGET).tex
	uplatex $(TARGET).tex
	dvipdfmx -p a4 -f ptex-ipa.map $(TARGET).dvi
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/default -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(FINAL_TARGET)_opt.pdf $(TARGET).pdf
	gs -q -dNOPAUSE -dBATCH -dPDFSETTINGS=/prepress -sDEVICE=pdfwrite -sOutputFile=$(FINAL_TARGET).pdf $(FINAL_TARGET)_opt.pdf
	rm -rf $(TARGET).log
	rm -rf $(TARGET).aux
	rm -rf $(TARGET).dvi
	rm -rf $(TARGET).pdf
	rm -rf $(FINAL_TARGET)_opt.pdf

clean:
	rm -rf $(TARGET).log
	rm -rf $(TARGET).aux
	rm -rf $(TARGET).dvi
	rm -rf $(TARGET).pdf
	rm -rf $(FINAL_TARGET)_opt.pdf
	rm -rf $(FINAL_TARGET).pdf
	rm -rf $(TARGET).bbl
	rm -rf $(TARGET).blg
	rm -rf missfont.log
