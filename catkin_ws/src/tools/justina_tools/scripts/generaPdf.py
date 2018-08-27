#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from generador_pdf.msg import Pdf
"""
    Pdf 
    tipo 1 -> inicio *** contenido->ruta del pdf
    tipo 2 -> texto  *** contenido->texto simple
    tipo 3 -> imagen *** contenido->ruta de la imagen
    tipo 4 -> fin    *** contenido->puede ir vacio
"""

from pdfdocument.document import PDFDocument
from reportlab.pdfgen import canvas
from reportlab.platypus import Image

def creaPdf(ruta):
	"""
	Crea un PDF simple en la ruta indicada
	"""
	global pdf
	pdf = PDFDocument(ruta)
	pdf.init_report()
	pdf.h1("Reporte de Test")
	pdf.hr()
	print "Guardando en " + ruta	

def guardaPdf(texto):
	"""
	Agrega texto al PDF
	"""
	global pdf
	rospy.loginfo('El dato a guardar es: %s', texto)
	pdf.p(texto)

def terminaPdf():
	"""
	Termina y genera el PDF
	"""
	global pdf
	pdf.generate()
	print "Terminando pdf"

def agregaImagenPDF(path):
	"""
	Agrega una imagen al PDF
	"""
	global pdf
	pdf.append(Image(path))

def callback(data):
	if data.tipo == 1:
		creaPdf(data.contenido)
	elif data.tipo == 2:
		guardaPdf(data.contenido)
	elif data.tipo == 3:
		agregaImagenPDF(data.contenido)
	elif data.tipo == 4:
		terminaPdf()
	else:
		print "Tipo de mensaje PDF no valido"

def listener(): 
	rospy.init_node('genera_pdf', anonymous=True)
	rospy.Subscriber('pdf_topic', Pdf, callback)
	rospy.spin()

if __name__ == '__main__':	
	listener()
