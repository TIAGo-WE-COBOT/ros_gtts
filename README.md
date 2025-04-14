# Text-to-speech in ROS

This package provides a node to perform text-to-speech using [gTTS](https://gtts.readthedocs.io/en/latest/module.html) as backend.

## Installation
* Open a terminal and move to the src folder of your workspace (here assumed to be ~/hri_ws)
    ```
    cd `~/hri_ws/src`
    ```
* Clone the repo with
    ```
    git clone https://github.com/TIAGo-WE-COBOT/ros_gtts.git
    ```
* Install `audio_play` and `audio_common_msgs` if you do not have them installed
    ```
    sudo apt-get install ros-noetic-audio-common
    ```
* Install the Python bindings for [gTTS](https://gtts.readthedocs.io/en/latest/module.html)
    ```
    pip install gTTS
    ```
* Build the package from the workspace root
    ```
    cd ~/hri_ws
    catkin build ros_gtts
    ```

## Usage
* Open a terminal and launch `gtts.launch`
    ```
    roslauch ros_gtts gtts.launch language:=it
    ```
    where the `language` argument is used to set the TTS language. Default language is English (*i.e.* `language:=en`).

> [!TIP]
> You can check the available languages and corresponding abbreviations by running 
> ```
>python -c "import gtts; [print(lang, '\t', name) for lang, name in gtts.lang.tts_langs().items()]"
> ```
> in your command line
    
* Open another terminal and publish to the `/tts/string_input` topic
    ```
    rostopic pub -1 /tts/string_input std_msgs/String "data: 'Ciao, piacere di conoscerti!'"
    ```
   to hear the synthesized speech.