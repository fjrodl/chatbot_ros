#!/bin/bash


STARS="**********************************************"

function yasmin_installation(){

    # clone
    cd ~/ros2_ws/src
    git clone https://github.com/uleroboticsgroup/yasmin.git

    # dependencies
    cd yasmin
    pip3 install -r requirements.txt

}



function audio_common_installation(){

    # clone
    cd ~/ros2_ws/src
    git clone https://github.com/mgonzs13/audio_common.git
    sudo apt install portaudio19-dev
    pip3 install -r audio_common/requirements.txt

}

function llama_installation(){

    # clone
    cd ~/ros2_ws/src
    git clone --recurse-submodules https://github.com/mgonzs13/llama_ros.git
    pip3 install -r llama_ros/requirements.txt

}


function tts_ros_installation(){

    # clone
    cd ~/ros2_ws/src
    git clone https://github.com/mgonzs13/audio_common.git
    git clone https://github.com/mgonzs13/tts_ros.git
    sudo apt install portaudio19-dev
    pip3 install -r audio_common/requirements.txt
    pip3 install -r tts_ros/requirements.txt

}


function whisper_installation(){

    #$ 
    cd ~/ros2_ws/src
 
    git clone --recurse-submodules https://github.com/mgonzs13/whisper_ros.git
    pip3 install -r whisper_ros/requirements.txt

}




function activate_nvidia(){
    
    cd ~/ros2_ws/src/whisper_ros/whisper_ros

    VAR_ORIGINAL="#option(WHISPER_CUBLAS"
    VAR_NEW="option(WHISPER_CUBLAS"

    cat CMakeLists.txt | grep WHISPER_CUBLAS
    sed -i '/$VAR_ORIGINAL/c\$VAR_NEW' CMakeLists.txt
    cat CMakeLists.txt | grep WHISPER_CUBLAS

    cd ~/ros2_ws/src/llama_ros/llama_ros
    sed -i '/#option(LLAMA_CUDA "llama: use CUDA" ON)/c\option(LLAMA_CUDA "llama: use CUDA" ON))' CMakeLists.txt
    sed -i '/#add_compile_definitions(GGML_USE_CUDA)/c\add_compile_definitions(GGML_USE_CUDA))' CMakeLists.txt

}

function deactivate_nvidia(){
    
    cd ~/ros2_ws/src/whisper_ros/whisper_ros

    sed -i '/option(WHISPER_CUBLAS "whisper: support for cuBLAS" ON)/c\#option(WHISPER_CUBLAS "whisper: support for cuBLAS" ON)' CMakeLists.txt
    
    cd ~/ros2_ws/src/llama_ros/llama_ros
    sed -i '/option(LLAMA_CUDA "llama: use CUDA" ON)/c\#option(LLAMA_CUDA "llama: use CUDA" ON))' CMakeLists.txt
    sed -i '/add_compile_definitions(GGML_USE_CUDA)/c\#add_compile_definitions(GGML_USE_CUDA))' CMakeLists.txt

}


function compilation(){
    
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build

}

function clean(){
    
    cd ~/ros2_ws
    rm -rf build install log

}
##################################################################
##################################################################
clear -x
echo -e "$STARS"
echo "Starting Installation of ROS 2 chatbot"
echo -e "$STARS"


clear -x
read -p "Do you want to install nvidia support for whisper_ros and llama_ros? (yes/no) " yn
case $yn in 
	yes ) echo ok, we will proceed;
		activate_nvidia;;
	no ) echo exiting...;
		break;;
	* ) echo invalid response;;
esac
echo -e "$STARS"
compilation

echo -e "$STARS"
echo "Installation ended"
echo -e "$STARS"