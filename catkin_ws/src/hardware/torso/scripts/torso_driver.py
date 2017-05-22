#Roboclaw1.Open(portName1, 38400)
#Roboclaw1.ResetEncoders(address1)
#Roboclaw1.BackwardM1(address2, right_rSpeed)
#Roboclaw1.ForwardM1(address2, -right_rSpeed)
#Roboclaw1.ReadEncM1(address1)
#Roboclaw1.Close()

import serial
import time
import threading
import random

class Torso(threading.Thread):
    def __init__(self,real):
        threading.Thread.__init__(self)
        print "Robot iniciado"
        self.real=not(real)
        self.dataOut=[0,0,0,0,0,0]
        self.dataIn=[0,0,0,0,0,0,0,0,0,0]

        self.runBase=True
        self.columna=[0,0.0,False,False,False]
        self.torso=  [0,0.0,False,False,False]#posDeseada,PosActual,enDeseado,enActual,SetPos
        self.hombro= [0,0.0,False,False,False]

    def SetSerial(self,port,baud):
        if self.real:
            self.ser=serial.Serial(port,baud,timeout=0.5)
            print "----------------puerto",port,baud,"--------"
    def ComBase(self):
        if self.real:   #col,tor,hom, encol,entor,enhom

            self.dataOut[0]=int(self.columna[0])
            self.dataOut[1]=int(self.torso[0])+90
            self.dataOut[2]=int(self.hombro[0])+20
            
            if self.columna[2]:
                self.dataOut[3]=ord('T')
            else:
                self.dataOut[3]=ord('F')

            if self.torso[2]:
                self.dataOut[4]=ord('T')
            else:
                self.dataOut[4]=ord('F')

            if self.hombro[2]:
                self.dataOut[5]=ord('T')
            else:
                self.dataOut[5]=ord('F')

            out="#O"
            crc=0
            for e in self.dataOut:
                out+=chr(e)
                crc+=e
            while crc>255:
                crc-=256
            out+=chr(crc)
            self.ser.write(out)
            self.ser.flush()#vaciar buffer del puerto serie
            time.sleep(0.01)
            buff=self.ser.readline()
            cad=buff.split(":")
            if cad[0]=="Robot" and len(cad)==2:
                c=cad[1].split(" ")
                if len(c)==9:
                    #c[0] colDeseado
                    self.columna[1]=float(c[1])
                    if int(c[2])==1:
                        self.columna[3]=True
                    else:
                        self.columna[3]=False
                    #c[3] torDeseado
                    self.torso[1]=float(c[4])
                    if int(c[5])==1:
                        self.torso[3]=True
                    else:
                        self.torso[3]=False
                    #c[6] homDeseado
                    self.hombro[1]=float(c[7])
                    if int(c[8])==1:
                        self.hombro[3]=True
                    else:
                        self.hombro[3]=False
        else:
            self.columna[1]=self.columna[0]
            self.columna[3]=self.columna[2]

            self.torso[1]=self.torso[0]
            self.torso[3]=self.torso[2]
            
            self.hombro[1]=self.hombro[0]
            self.hombro[3]=self.hombro[2]



    def setPos(self):
        if self.real:
            send=False
            if self.columna[4]:

                self.dataOut[0]=ord('C')
                self.dataOut[1]=int(self.columna[0])
            
                self.columna[4]=False
                send=True

            elif self.torso[4]:

                self.dataOut[0]=ord('T')
                self.dataOut[1]=int(self.torso[0])+90
            
                self.torso[4]=False
                send=True

            elif self.hombro[4]:

                self.dataOut[0]=ord('H')
                self.dataOut[1]=int(self.hombro[0])+20

                self.hombro[4]=False
                send=True


            if send:
                self.dataOut[2]=0
                self.dataOut[3]=0
                self.dataOut[4]=0
                self.dataOut[5]=0
                out="#S"
                crc=0
                for e in self.dataOut:
                    out+=chr(e)
                    crc+=e
                while crc>255:
                    crc-=256
                out+=chr(crc)
                self.ser.write(out)
                self.ser.flush()#vaciar buffer del puerto serie
                time.sleep(0.01)
        else:
            if self.columna[4]:
                self.columna[1]=self.columna[0]
                self.columna[4]=False

            elif self.torso[4]:
                self.torso[1]=self.torso[0]
                self.torso[4]=False

            elif self.hombro[4]:
                self.hombro[1]=self.hombro[0]
                self.hombro[4]=False

    def run(self):
        while self.runBase:#rospy.is_shutdown():#
            self.ComBase()
            time.sleep(0.01)
            self.setPos()
