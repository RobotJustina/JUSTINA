import rospy
#import speech_recognition as SR
from threading import Lock

from audio_common_msgs.msg import AudioData

class AudioStream(object):
        def __init__(self, topic_name="audio", buffer_size=10240):
            self.buffer_size = buffer_size
            self.topic_name = topic_name
            self.lock = Lock()
            self.buffer = bytes()
            self.sub_audio = rospy.Subscriber(
                self.topic_name, AudioData, self.audio_cb)

        def read_once(self, size):
            with self.lock:
                buf = self.buffer[:size]
                self.buffer = self.buffer[size:]
                return buf

        def read(self, size):
            while not rospy.is_shutdown() and len(self.buffer) < size:
                rospy.sleep(0.001)
            return self.read_once(size)

        def close(self):
            try:
                self.sub_audio.unregister()
            except:
                pass
            self.buffer = bytes()

        def audio_cb(self, msg):
            with self.lock:
                self.buffer += bytes(msg.data)
                overflow = len(self.buffer) - self.buffer_size
                if overflow > 0:
                    self.buffer = self.buffer[overflow:]




if __name__ == '__main__':
    rospy.init_node('speech_test')
    stream = AudioStream("audio", 10240)
    stream.read
    rospy.spin()