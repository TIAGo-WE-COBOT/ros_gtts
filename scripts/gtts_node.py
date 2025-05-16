#!/usr/bin/env python

import argparse
import io
import gtts

import rospy
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData

class TTSNode:
    def __init__(self, language="en"):
        self.is_speaking = False
        self.language = language

        self.string_sub = rospy.Subscriber('string_input',
                                                String,
                                                self.speak)
        self.audio_data_pub = rospy.Publisher('audio_data_output',
                                               AudioData,
                                               queue_size=10)
        self.is_speaking_pub = rospy.Publisher('is_speaking',
                                              Bool,
                                              queue_size=1)

    def speak(self, msg):
        tts = gtts.gTTS(msg.data, lang=self.language)
        # Store in a BytesIO buffer
        audio_buffer = io.BytesIO()
        tts.write_to_fp(audio_buffer)
        audio_buffer.seek(0)  # rewind buffer
        # Estimate length of audio
        sample_rate = 16000
        channels = 1
        duration = len(audio_buffer.getvalue()) / (sample_rate * channels)
        #rospy.loginfo("Estimated duration of audio: {} seconds".format(duration))
        # TODO. Get sample rate from audio info
        # Update state
        self.is_speaking = True
        # Publish as AudioData
        audio_msg = AudioData()
        audio_msg.data = audio_buffer.read()
        self.audio_data_pub.publish(audio_msg)
        # Set timer to reset `is_speaking`
        # TODO. Understand why the duration computed above is half of the actual one... Data format reason?
        self.is_speaking_timer = rospy.Timer(rospy.Duration(duration*2 + 2.0), 
                                             self.is_speaking_timeout,
                                             oneshot=True
                                             )
    
    def is_speaking_timeout(self, event):
        self.is_speaking = False
        self.is_speaking_timer.shutdown()

if __name__ == "__main__":
    parser=argparse.ArgumentParser(
        description='A ROS node to perform text-to-speech with gTTS backend.'
    )
    parser.add_argument('-l', '--language', type=str,
                        default='en',
                        help='The abbreviation of the TTS language.\nAvailable languages and corresponding abbreviations can be listed by running\n\tpython -c "import gtts; [print(lang, \'\t\', name) for lang, name in gtts.lang.tts_langs().items()]"')
    rospy.init_node("gtts_node")
    args = parser.parse_args(rospy.myargv()[1:])
    tts_node = TTSNode(language=args.language)
    r = rospy.Rate(10)  # 10hz
    try:
        while not rospy.is_shutdown():
            tts_node.is_speaking_pub.publish(Bool(tts_node.is_speaking))
            r.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down on user request...")