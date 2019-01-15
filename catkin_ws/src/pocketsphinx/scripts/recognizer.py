#!/usr/bin/env python

import argparse
import rospy
import rospkg

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio

from std_msgs.msg import String, Bool 
from std_srvs.srv import *
from knowledge_msgs.msg import SphinxSetFile
from hri_msgs.msg import RecognizedSpeech
import os
import commands

class recognizer(object):

    def callbackSetKws(self, data):
        print "SET Keyphrase file for SPEACH"
        self.decoder.set_kws(data.id, data.file_path)
    
    def callbackSetJsgf(self, data):
        print "SET Grammar file for SPEACH"
        self.decoder.set_jsgf_file(data.id, data.file_path)

    def callbackSetSearch(self, data):
        print "SET the SEARCH TYPE"
        self.decoder.end_utt()
        self.decoder.set_search(data.data)
        self.decoder.start_utt()

    def callbackSetMic(self, data):
        if data.data:
            print "Enable MIC"
            self.enable_mic = True 
        else:
            print "Disable MIC"
            self.enable_mic = False 

    def __init__(self):

        # initialize ROS
        self.speed = 0.2

        # Start node
        rospy.init_node("recognizer")
        
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber("/pocketsphinx/set_kws", SphinxSetFile, self.callbackSetKws)
        rospy.Subscriber("/pocketsphinx/set_jsgf", SphinxSetFile, self.callbackSetJsgf)
        rospy.Subscriber("/pocketsphinx/set_search", String, self.callbackSetSearch)
        rospy.Subscriber("/pocketsphinx/mic", Bool, self.callbackSetMic)

        self._lm_param = "~lm"
        self._dict_param = "~dict"
        self._kws_param = "~kws"
        self._stream_param = "~stream"
        self._wavpath_param = "~wavpath"

        # you may need to change publisher destination depending on what you run
        self.pub_ = rospy.Publisher('~output', String, queue_size=1)
        self.pubRecognizedSpeech = rospy.Publisher('/recognizedSpeech', RecognizedSpeech, queue_size=1)

        if rospy.has_param(self._lm_param):
            self.lm = rospy.get_param(self._lm_param)
        else:
            rospy.loginfo("Loading the default acoustic model")
            self.lm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
            #self.lm = "/home/rey/docker_volumen/model_size-256_layers-3_filter-512_best"
            rospy.loginfo("Done loading the default acoustic model")

        if rospy.has_param(self._dict_param):
            self.lexicon = rospy.get_param(self._dict_param)
        else:
            rospy.logerr('No dictionary found. Please add an appropriate dictionary argument.')
            return

        if rospy.has_param(self._kws_param):
            self.kw_list = rospy.get_param(self._kws_param)
        else:
            rospy.logerr('kws cant run. Please add an appropriate keyword list file.')
            return

        if rospy.has_param(self._stream_param):
            self.is_stream = rospy.get_param(self._stream_param)
            if not self.is_stream:
                if rospy.has_param(self._wavpath_param):
                    self.wavpath = rospy.get_param(self._wavpath_param)
                    if self.wavpath == "none":
                        rospy.logerr('Please set the wav path to the correct file location')
                else:
                    rospy.logerr('No wav file is set')
        else:
            rospy.logerr('Audio is not set to a stream (true) or wav file (false).')
            self.is_stream = rospy.get_param(self._stream_param)

        self.start_recognizer()

    def start_recognizer(self):
        rospack = rospkg.RosPack()
        sphinx_path = rospack.get_path('pocketsphinx')
        # initialize pocketsphinx. As mentioned in python wrapper
        rospy.loginfo("Initializing pocketsphinx")
        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

        # Hidden Markov model: The model which has been used
        config.set_string('-hmm', self.lm)
        # Pronunciation dictionary used
        config.set_string('-dict', self.lexicon)
        # Keyword list file for keyword searching
        #config.set_string('-kws', self.kw_list)
        #config.set_string('-lm', sphinx_path + '/vocab/3357.lm.bin')
        config.set_string('-jsgf', sphinx_path + '/vocab/restaurant/restaurant.jsgf')


        rospy.loginfo("Opening the audio channel")

        if not self.is_stream:
            self.decoder = Decoder(config)
            self.decoder.start_utt()
            try:
                wavFile = open(self.wavpath, 'rb')
            except:
                rospy.logerr('Please set the wav path to the correct location from the pocketsphinx launch file')
                rospy.signal_shutdown()
            # Update the file link above with relevant username and file
            # location
            in_speech_bf = False
            while not rospy.is_shutdown():
                buf = wavFile.read(1024)
                if buf:
                    self.decoder.process_raw(buf, False, False)
                else:
                    break
            self.decoder.end_utt()
            hypothesis = self.decoder.hyp()
            if hypothesis == None:
                rospy.logwarn("Error, make sure your wav file is composed of keywords!!")
                rospy.logwarn("Otherwise, your speech is uninterpretable :C ")
            else:
                print hypothesis.hypstr

        else:
	    # Pocketsphinx requires 16kHz, mono, 16-bit little-Endian audio.
	    # See http://cmusphinx.sourceforge.net/wiki/tutorialtuning
            stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=2048)
            stream.start_stream()
            rospy.loginfo("Done opening the audio channel")

            #decoder streaming data
            rospy.loginfo("Starting the decoder")
            self.decoder = Decoder(config)
            self.decoder.start_utt()
            self.enable_mic = True 
            utt_started = False
            rospy.loginfo("Done starting sphinx speech recognition by Julio Cruz")

            # Main loop
            while not rospy.is_shutdown():
                # taken as is from python wrapper
                buf = stream.read(2048)
                if buf and self.enable_mic:
                    self.decoder.process_raw(buf, False, False)
                    in_speech = self.decoder.get_in_speech()
                    if in_speech and not(utt_started):
                        utt_started = True 
                        rospy.loginfo("Listening....")
                    if not(in_speech) and utt_started:
                        self.decoder.end_utt()
                        self.publish_result()
                        self.decoder.start_utt()
                        utt_started = False 
                        rospy.loginfo("Ready....")

    def publish_result(self):
        """
        Publish the words
        """
        if self.decoder.hyp() != None:
            print 'Decoder: ' + self.decoder.hyp().hypstr
            hypotesis = [self.decoder.hyp().hypstr.lower()]
            confidence = [0.999]
            request = RecognizedSpeech(hypotesis, confidence)
            self.pubRecognizedSpeech.publish(request)
            #print ([(seg.word) 
            #    for seg in self.decoder.seg()])
            #seg.word = seg.word.lower()
            #self.decoder.end_utt()
            #self.decoder.start_utt()
            #self.pub_.publish(seg.word)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stopping PocketSphinx")

if __name__ == "__main__":
    if len(sys.argv) > 0:
        start = recognizer()
