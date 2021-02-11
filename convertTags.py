from fpdf import FPDF # God-tier library
import glob

pdf = FPDF('P', "in", "Letter") # Portrait, inch, letter
pdf.set_margins(0, 0, 0)
pdf.add_page()

isSecond = False
for filename in glob.glob("./tags/*.jpg"):
    if isSecond:
        pdf.image(filename, .25, 5.75, 5)
        pdf.add_page()
    else:
        pdf.image(filename, .25, .25, 5)
    isSecond = not isSecond
    
pdf.output("formattedTags.pdf", 'F')