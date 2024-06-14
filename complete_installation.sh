#!/bin/bash


STARS="**********************************************"

ROS2_FOLDER_PATH=~/ros2_ws
ROS2_SRC_PATH=$ROS2_FOLDER_PATH/src


# Function to check if a ROS2  folder exists and create it if it doesn't
ensure_directory_exists() {
  local dir_path="$1"
  
  if [ -d "$dir_path" ]; then
    echo "Directory $dir_path already exists."
  else
    echo "Directory $dir_path does not exist. Creating it now..."
    mkdir -p "$dir_path"
    if [ $? -eq 0 ]; then
      echo "Directory $dir_path created successfully."
    else
      echo "Failed to create directory $dir_path."
    fi
  fi
}


function yasmin_installation(){

    # clone
    cd $ROS2_SRC_PATH
    git clone https://github.com/uleroboticsgroup/yasmin.git

    # dependencies
    cd yasmin
    pip3 install -r requirements.txt

}



function audio_common_installation(){

    # clone
    cd $ROS2_SRC_PATH
    git clone https://github.com/mgonzs13/audio_common.git
    sudo apt install -y portaudio19-dev
    pip3 install -r audio_common/requirements.txt

}

function llama_installation(){

    # clone
    cd $ROS2_SRC_PATH
    git clone --recurse-submodules https://github.com/mgonzs13/llama_ros.git
    pip3 install -r llama_ros/requirements.txt

}


function tts_ros_installation(){

    # clone
    cd $ROS2_SRC_PATH
    git clone https://github.com/mgonzs13/audio_common.git
    git clone https://github.com/mgonzs13/tts_ros.git
    sudo apt install -y portaudio19-dev
    pip3 install -r audio_common/requirements.txt
    pip3 install -r tts_ros/requirements.txt

}


function whisper_installation(){

    #$ 
    cd $ROS2_SRC_PATH
 
    git clone --recurse-submodules https://github.com/mgonzs13/whisper_ros.git
    pip3 install -r whisper_ros/requirements.txt

}




function activate_nvidia(){
    
    cd $ROS2_SRC_PATH/whisper_ros/whisper_ros

    VAR_ORIGINAL="#option(WHISPER_CUBLAS "whisper: support for cuBLAS" ON)"
    VAR_NEW="option(WHISPER_CUBLAS "whisper: support for cuBLAS" ON)"

    cat CMakeLists.txt | grep WHISPER_CUBLAS
    sed -i '/$VAR_ORIGINAL/c\$VAR_NEW' CMakeLists.txt
    cat CMakeLists.txt | grep WHISPER_CUBLAS

    cd $ROS2_SRC_PATH/llama_ros/llama_ros
    sed -i '/#option(LLAMA_CUDA "llama: use CUDA" ON)/c\option(LLAMA_CUDA "llama: use CUDA" ON))' CMakeLists.txt
    sed -i '/#add_compile_definitions(GGML_USE_CUDA)/c\add_compile_definitions(GGML_USE_CUDA))' CMakeLists.txt

}

function deactivate_nvidia(){
    
    cd $ROS2_SRC_PATH/whisper_ros/whisper_ros

    sed -i '/option(WHISPER_CUBLAS "whisper: support for cuBLAS" ON)/c\#option(WHISPER_CUBLAS "whisper: support for cuBLAS" ON)' CMakeLists.txt
    
    cd $ROS2_SRC_PATH/llama_ros/llama_ros
    sed -i '/option(LLAMA_CUDA "llama: use CUDA" ON)/c\#option(LLAMA_CUDA "llama: use CUDA" ON))' CMakeLists.txt
    sed -i '/add_compile_definitions(GGML_USE_CUDA)/c\#add_compile_definitions(GGML_USE_CUDA))' CMakeLists.txt

}


function compilation(){
    
    cd $ROS2_FOLDER_PATH
    rosdep install --from-paths src --ignore-src -r -y
    colcon build

}

function clean(){
    
    cd $ROS2_FOLDER_PATH
    rm -rf build install log

}
##################################################################
##################################################################
clear -x
echo -e "$STARS"
echo "Starting Installation of ROS 2 chatbot"
echo -e "$STARS"

ensure_directory_exists $ROS2_SRC_PATH

yasmin_installation

echo -e "$STARS"
echo -e "$STARS"
llama_installation

echo -e "$STARS"
echo -e "$STARS"
whisper_installation

echo -e "$STARS"
echo -e "$STARS"
tts_ros_installation

echo -e "$STARS"
echo -e "$STARS"
audio_common_installation


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