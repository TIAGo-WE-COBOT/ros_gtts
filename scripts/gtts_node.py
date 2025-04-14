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
        
        # Publish as AudioData
        audio_msg = AudioData()
        audio_msg.data = audio_buffer.read()
        self.audio_data_pub.publish(audio_msg)

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
    try:
        while not rospy.is_shutdown():
            pass
    except KeyboardInterrupt:
        print("Shutting down on user request...")