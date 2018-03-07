#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from generador_pdf.msg import Pdf

def mandaInformacion(ruta):
    pub = rospy.Publisher('pdf_topic', Pdf, queue_size=10)
    rospy.init_node('envia_informacion', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    contador = 1
    print "Comenzando"
    """
    Pdf 
    tipo 1 -> inicio *** contenido->ruta del pdf
    tipo 2 -> texto  *** contenido->texto simple
    tipo 3 -> imagen *** contenido->ruta de la imagen
    tipo 4 -> fin    *** contenido->puede ir vacio
    """
    mensaje = Pdf()
    mensaje.tipo = 1
    mensaje.contenido = ruta
    rospy.loginfo(mensaje)
    pub.publish(mensaje)
    while not rospy.is_shutdown():
        mensaje.tipo = 2
        mensaje.contenido = "Datos %s" % contador
        rospy.loginfo(mensaje)
        pub.publish(mensaje)
        contador = contador + 1
        if contador==100:
            mensaje.tipo = 4
            mensaje.contenido = ""
            rospy.loginfo(mensaje)
            pub.publish(mensaje)
            return
        rate.sleep()
    print "terminando"
    mensaje.tipo = 4
    mensaje.contenido = ""
    rospy.loginfo(mensaje)
    pub.publish(mensaje)

if __name__ == '__main__':
    try:
        mandaInformacion("prueba1.pdf")
        mandaInformacion("prueba2.pdf")
    except rospy.ROSInterruptException:        
        pass
