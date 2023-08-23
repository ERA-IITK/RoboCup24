
# **Send_data**

File to send data (coordinates of the detection box using IMX219 camera sensor) from Rpi to NUC.

The image generated is in full fov at max resolution


### Installing Dependencies
    sudo apt install python3-pip
####
    pip install sockets
####
    pip install picamera2
####
    pip3 install numpy
####
    pip install opencv-python
####

### Working of code :
The library used in picamera2, since opencv deos not give desired results.
The code first establishes a connection between NUC and Rpi using sockets, once connected the terminal outputs the message `Connected to server`

#### **preview_config**
It creates a video with specific configuration.
- **size**: &nbsp;  The resolution of video generated
- **format**:  &nbsp; Specifies the color channels of the video generated, current it is set to RGB888 (i.e. RGB), other modes include 'YUV888' for greyscale etc.
- **raw**:  &nbsp; Gives raw video obtained without buffer. The camera can be set in 7 sensor modes, by default it is set to 0, which gives almost 55% less fov. Modes 5,7 give full fov. Mode 5 also retains video quality and resolution of the video.

#### **Sending data**
The image obtained must be converted to string before serializing it. Hence it is converted to string and then sent over to the processing unit i.e NUC. 

The string is then decoded in the NUC and coordinates are obtained.
