#!/usr/bin/env python
import pyaudio
import struct
import time
import gc
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from audio_msgs.srv import *
from rospy.numpy_msg import numpy_msg
import numpy as np
import threading
from threading import Thread



class audioCap(Thread):

    def __init__(self, index):
        self.data = None
        self.index = index
        super(audioCap, self).__init__()

    def run(self):
        print 'record ',self.index
        FORMAT = pyaudio.paFloat32
        CHANNELS = 1
        RATE = 44100
        FRAMESIZE = 1024
        NOFFRAMES = 2

        INDEX = self.index
        p = pyaudio.PyAudio()

        stream = p.open(format=FORMAT, channels=CHANNELS,
                        rate=RATE, input=True,
                        frames_per_buffer=FRAMESIZE,
                        output_device_index=INDEX, input_device_index=INDEX)
        data = stream.read(NOFFRAMES*FRAMESIZE)
        decoded = np.fromstring(data, 'Float32');

        stream.stop_stream()
        stream.close()
        p.terminate()

        print('stop')

        self.data = decoded


class Beanforming(Thread):

    def __init__(self, index, nphones, newsamples, arrayDataPhone, sound_speed, spacing, look_dirs, sampling_rate, time_delays, fft_freqs, spacial_filt, bf_data):
        self.index = index
        self.arrayDataPhone = arrayDataPhone
        self.nphones = nphones
        self.newsamples = newsamples
        self.sound_speed = sound_speed
        self.sampling_rate = sampling_rate
        self.spacing = spacing
        self.look_dirs = look_dirs
        self.bf_data = []
        self.phone_data = []
        self.time_delays = time_delays
        self.fft_freqs = fft_freqs
        self.spacial_filt = spacial_filt
        self.bf_data = bf_data
        self.Ipeaks = []
        self.Ipeak = 0.0
        self.Vpeaks = []
        self.Vpeak = 0.0
        super(Beanforming, self).__init__()

    def run(self):
        print "corriendo hilo"
        try :

            phone_data=getDataPhone(self.nphones,self.newsamples,self.arrayDataPhone)

            #'''function to do conventional beamforming'''
            # allocate space to put data
            bf_data = np.zeros((self.newsamples, len(self.look_dirs)))
            # find time lags between phones and the bf matrix
            time_delays = np.matrix( (self.spacing/self.sound_speed))
            fft_freqs = np.matrix(np.linspace( 0, self.sampling_rate, self.newsamples, endpoint=False)).transpose()
            for ind, direction in enumerate(self.look_dirs):
                spacial_filt = 1.0/self.nphones*np.exp(-2j*np.pi*fft_freqs*time_delays*np.cos(direction))
                # fft the data, and let's beamform.
                bf_data[:,ind] = np.sum(np.fft.irfft( np.fft.fft(phone_data,self.newsamples,0)*np.array(spacial_filt), \
                 self.newsamples, 0), 1)

            self.phone_data = phone_data

            tempsignal = (sum(abs(np.fft.fft( bf_data, self.newsamples, 0))**2/self.newsamples**2, 0)).tolist()
            tempVpeak = max(tempsignal)
            tempIpeak = self.look_dirs[tempsignal.index(tempVpeak)]

            self.Ipeak = tempIpeak
            print "Termino hilo"
        except :
            print "Fallo hilo"
            self.Ipeak = 0.0

       



def energyDetect(nphone_r,data,threshold,percent):

    flag = True
    size = np.size(data)
    per = (int)(size*percent/3)
    cont = 0
    for k in range (0,size,5) :
        if  abs(data[k]) > threshold :
            cont = cont+1

    if cont>per :
        flag=True
    else :
        flag=False
        
    return flag




def getDataPhone(nphones,samplesX,dataIn):

    arrayData = []
    for x in range(0,samplesX,1):
        data = []
        for y in range(0,nphones,1):
            data.append(dataIn[y*samplesX+x])
        arrayData.append(data)
    
    return arrayData



def correctMic(correctionI, correctionD, data):
    return data[correctionI:(np.size(data)-correctionD-1)]




def callbackStart(data):
    rospy.loginfo("Reciving data:" + str(data))

    #time.sleep(1)
    #gc.collect()
    global newMic1, newMic2, newMic3

    newMic1 = []
    newMic2 = []
    newMic3 = []

    i = 0

    while i < 160 :
        i = i + 1
        print "intentando captura"
        mic3 = audioCap(4)
        mic2 = audioCap(6)
        mic1 = audioCap(5)
        mic3.start()
        mic2.start()
        mic1.start()
        mic3.join()
        mic2.join()
        mic1.join()

        if i<150 and energyDetect(nphones_r,mic1.data,0.05,0.1) and energyDetect(nphones_r,mic2.data,0.05,0.1)\
           and energyDetect(nphones_r,mic3.data,0.05,0.1) :

            newMic1=correctMic(0,0,mic1.data)
            newMic2=correctMic(0,0,mic2.data)
            newMic3=correctMic(0,0,mic3.data)

            i = 160  

            print "XXXXX termino topico XXXXXX"
            #gc.collect()

    

    



    
def handle(request):

    global newMic1, newMic2, newMic3
    print "XXXXX Inicia Servicio XXXXXX"

    # Arreglo Horizontal
    arrayDataPhone=np.concatenate((newMic1,newMic2),axis=0)
    bfm = Beanforming(1, nphones, newsamples, arrayDataPhone, sound_speed, spacing, look_dirs, sampling_rate, time_delays_H, fft_freqs_H, spacial_filt_H, bf_data_H)
    bfm.start()


    # phone_data=getDataPhone(nphones,newsamples,arrayDataPhone)
    # '''function to do conventional beamforming'''
    # # allocate space to put data
    # bf_data = np.zeros((newsamples, len(look_dirs)))
    # # find time lags between phones and the bf matrix
    # time_delays = np.matrix( (spacing/sound_speed))
    # fft_freqs = np.matrix(np.linspace( 0, sampling_rate, newsamples, endpoint=False)).transpose()
    # try :
    #     for ind, direction in enumerate(look_dirs):
    #         spacial_filt = 1.0/nphones*np.exp(-2j*np.pi*fft_freqs*time_delays*np.cos(direction))
    #         # fft the data, and let's beamform.
    #         bf_data[:,ind] = np.sum(np.fft.irfft( np.fft.fft(phone_data,newsamples,0)*np.array(spacial_filt), \
    #          newsamples, 0), 1)

    #     tempsignal = (sum(abs(np.fft.fft( bf_data, newsamples, 0))**2/newsamples**2, 0)).tolist()
    #     tempVpeak = max(tempsignal)
    #     tempIpeak = look_dirs[tempsignal.index(tempVpeak)]
    # except :
    #     tempIpeak = 0

    # Ipeak = tempIpeak







    # Arreglo Vertical
    arrayDataPhone2=np.concatenate((newMic1,newMic3),axis=0)
    bfmD = Beanforming(2, nphones, newsamples, arrayDataPhone2, sound_speed, spacing, look_dirs2, sampling_rate, time_delays_V, fft_freqs_V, spacial_filt_V, bf_data_V)
    bfmD.start()
    

    # phone_data=getDataPhone(nphones,newsamples,arrayDataPhone2)
    # '''function to do conventional beamforming'''
    # # allocate space to put data
    # bf_data = np.zeros((newsamples, len(look_dirs2)))
    # # find time lags between phones and the bf matrix
    # time_delays = np.matrix( (spacing/sound_speed))
    # fft_freqs = np.matrix(np.linspace( 0, sampling_rate, newsamples, endpoint=False)).transpose()
    # try :
    #     for ind, direction in enumerate(look_dirs2):
    #         spacial_filt = 1.0/nphones*np.exp(-2j*np.pi*fft_freqs*time_delays*np.cos(direction))
    #         # fft the data, and let's beamform.
    #         bf_data[:,ind] = np.sum(np.fft.irfft( np.fft.fft(phone_data,newsamples,0)*np.array(spacial_filt), \
    #          newsamples, 0), 1)
    #     tempsignal = (sum(abs(np.fft.fft( bf_data, newsamples, 0))**2/newsamples**2, 0)).tolist()
    #     tempVpeak = max(tempsignal)
    #     tempIpeak = look_dirs2[tempsignal.index(tempVpeak)]

    # except :
    #     tempIpeak = 0
    
    # IpeakD = tempIpeak



    bfm.join()
    bfmD.join()


    Ipeak = bfm.Ipeak
    IpeakD = bfmD.Ipeak

    print "direction" , IpeakD
    

    if Ipeak == 0 :
        Ipeak = 0
    elif IpeakD>(np.pi/2)-0.4:
        Ipeak = Ipeak - (np.pi/2)
    elif IpeakD<(np.pi/2)-.3 :
        if Ipeak > np.pi/2 : 
            Ipeak = (np.pi)-(Ipeak-(np.pi/2))
        else :
            Ipeak = (-np.pi)-(Ipeak-(np.pi/2))


    # if Ipeak == 0 :
    #     Ipeak = 0
    # elif IpeakD<(np.pi/2)-0.3 :
    #     Ipeak = (np.pi/2)-Ipeak
    # elif IpeakD>(np.pi/2)-0.3 :
    #     if Ipeak > np.pi/2 : 
    #         Ipeak = (-1*np.pi)-((np.pi/2)-Ipeak)
    #     else :
    #         Ipeak = (np.pi)-((np.pi/2)-Ipeak)



    print "Angulo obtenido ",Ipeak

    return srvAngleResponse(float(Ipeak))



    

def main():
    global pub_angles, pub_flag
    rospy.init_node('audio_source')
    print "Init Node"
    pub_angles = rospy.Publisher('/audio_source/angles',Float32MultiArray ,queue_size = 1)
    pub_flag = rospy.Publisher('/audio_source/flag', Bool, queue_size = 1)
    rospy.Subscriber("/audio_source/start", Bool, callbackStart)
    print "Init Service"
    s = rospy.Service('audio_msgs/srv_Angle', srvAngle, handle)

    rospy.spin()



    

if __name__ == '__main__':
    global newMic1, newMic2, newMic3

    nphones_r = 3 #32 phones
    nphones = 2 
    sound_speed = 343 #meters per second sound speed
    spacing = np.linspace(0, .15, nphones) #first and second phone 2 m apart
    look_dirs = np.arccos(np.linspace(-1, 1, 180)) #lokiing dirs for search
    look_dirs2 = np.arccos(np.linspace(-1, 1, 180/2))
    samples = 1024*2 #number of samples
    sampling_rate = 44100 #100 hz sampling rate
    correction = 10 #correccion


    
    newsamples = samples-(0)-1
    
    # Horizontal
    bf_data_H = np.zeros((newsamples, len(look_dirs)))
    time_delays_H = np.matrix( (spacing/sound_speed))
    fft_freqs_H = np.matrix(np.linspace( 0, sampling_rate, newsamples, endpoint=False)).transpose()
    for ind, direction in enumerate(look_dirs):
        spacial_filt_H = 1.0/nphones*np.exp(-2j*np.pi*fft_freqs_H*time_delays_H*np.cos(direction))
    
    #Vertical
    bf_data_V = np.zeros((newsamples, len(look_dirs2)))
    time_delays_V = np.matrix( (spacing/sound_speed))
    fft_freqs_V = np.matrix(np.linspace( 0, sampling_rate, newsamples, endpoint=False)).transpose()
    for ind, direction in enumerate(look_dirs2):
        spacial_filt_V = 1.0/nphones*np.exp(-2j*np.pi*fft_freqs_V*time_delays_V*np.cos(direction))

    
    
    newMic1 = []
    newMic2 = []
    newMic3 = []


    pa = pyaudio.PyAudio()
    chosen_device_index = -1
    for x in xrange(0,pa.get_device_count()):
        info = pa.get_device_info_by_index(x)
        print pa.get_device_info_by_index(x)
        if info["name"] == "pulse":
            chosen_device_index = info["index"]
            print "Chosen index: ", chosen_device_index




    main()
