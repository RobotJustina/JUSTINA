#!/usr/bin/env python

import argparse
import rospy
import rospkg

import time

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio

import time

from std_msgs.msg import String, Bool, Int8 
from std_srvs.srv import *
from hri_msgs.msg import SphinxSetFile, SphinxSetSearch
from hri_msgs.msg import RecognizedSpeech
import os
import commands

"""
RESPEAKER_RATE = 16000
RESPEAKER_CHANNELS = 1 # change base on firmwares, 1_channel_firmware.bin as 1 or 6_channels_firmware.bin as 6
RESPEAKER_WIDTH = 2
# run getDeviceInfo.py to get index
RESPEAKER_INDEX = 2  # refer to input device id
CHUNK = 1024
"""
class recognizer(object):

    def callbackSetKws(self, data):
        print "SET Keyphrase file for SPEACH"
        self.decoder.set_kws(data.id, data.file_path)
    
    def callbackSetJsgf(self, data):
        print "SET Grammar file for SPEACH"
        self.decoder.set_jsgf_file(data.id, sphinx_path + "/" + data.file_path)

    def callbackSetSearch(self, data):
        print "SET the SEARCH TYPE"
        self.decoder.end_utt()
        self.decoder.set_search(data.data)
        self.decoder.start_utt()

    def callbackSetSearchAndTime(self, data):
        print "Set SEARCH TYPE and TIME of recognition"
        global reco_time
        if self.enable_mic:
            self.decoder.end_utt()
        self.decoder.set_search(data.search_id)
        reco_time = rospy.Duration.from_sec(data.recognitionTime)
        self.decoder.start_utt()
        self.enable_mic = True


    def __init__(self):

        # initialize ROS
        self.speed = 0.2

        # Start node
        rospy.init_node("recognizer_with_respeaker")
        
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber("/pocketsphinx/set_kws", SphinxSetFile, self.callbackSetKws)
        rospy.Subscriber("/pocketsphinx/set_jsgf", SphinxSetFile, self.callbackSetJsgf)
        rospy.Subscriber("/pocketsphinx/set_search", SphinxSetSearch, self.callbackSetSearchAndTime)

        

        self._lm_param = "~lm"
        self._dict_param = "~dict"
        self._kws_param = "~kws"
        self._jsgf_param = "~jsgf"
        self._stream_param = "~stream"
        self._wavpath_param = "~wavpath"

	# respeaker param

	self._rate_param = rospy.get_param("~rate", 16000)
	self._channel_param = rospy.get_param("~channel", 1)
	self._width_param = rospy.get_param("~width", 2L)
	self._index_param = rospy.get_param("~index", 2)
	self._chunk_param = rospy.get_param("~chunk", 2048)

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
        
        if rospy.has_param(self._jsgf_param):
            self._jsgf_param = rospy.get_param(self._jsgf_param)
        else:
            rospy.logerr('jsgf cant run. Please add an appropriate grammar file.')
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
        global sphinx_path
        global reco_time
 
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
        config.set_string('-jsgf', self._jsgf_param)

        config.set_string('-logfn', '/dev/null')
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
            p = pyaudio.PyAudio()
            stream = p.open(format=p.get_format_from_width(self._width_param), 
			    channels=self._channel_param,
                            rate=self._rate_param, 
                            input=True, 
		            frames_per_buffer=self._chunk_param, 
                            input_device_index=self._index_param)

            stream.start_stream()
            rospy.loginfo("Done opening the audio channel")

            #decoder streaming data
            rospy.loginfo("Starting the decoder")
            self.decoder = Decoder(config)
            self.decoder.start_utt()
            self.enable_mic = True 
            utt_started = False
            rospy.loginfo("Done starting sphinx speech recognition by Julio Cruz")
            #elapsed = 0.0
            start = rospy.Duration.from_sec(0.0)
            end = rospy.Duration.from_sec(0.0)
            reco_time = rospy.Duration.from_sec(0.0) #2.0
            # Main loop
            while not rospy.is_shutdown():
                # taken as is from python wrapper
                buf = stream.read(self._chunk_param)
                if buf and self.enable_mic:
                    self.decoder.process_raw(buf, False, False)
                    in_speech = self.decoder.get_in_speech()
                    if in_speech and not(utt_started):
                        start = rospy.get_rostime() #time.time()
                        utt_started = True
                        #print 'ROS time: ' + str(rospy.get_rostime())
                        rospy.loginfo("Listening....")
                    end = rospy.get_rostime() #time.time()
                    elapsed = end - start
                    if reco_time == rospy.Duration.from_sec(0.0):
                        elapsed = rospy.Duration.from_sec(0.0)
                        #print 'RECO TIME: ' + str(reco_time)
                    if (not(in_speech) or elapsed > reco_time) and utt_started:
                        print 'Time elapsed: ' + str(elapsed)
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
            #print 'Decoder: ' + str(self.decoder.hyp().best_score)
            hypotesis = [self.decoder.hyp().hypstr.lower()]
            confidence = [0.999]
            request = RecognizedSpeech(hypotesis, confidence)
            self.pubRecognizedSpeech.publish(request)
            #print ([(seg.word + ' ' + str(seg.prob)) 
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
