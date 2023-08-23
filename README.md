# RoboMime
A program to make a robot imitate the skeletal pose of a human operator using a Microsoft Azure Kinect depth camera and Robovie robot.

## Prerequisites

**This program has only been tested on Windows, but should work on Linux as well. The Microsoft Azure Kinect SDKs are currently only supported on Windows and Linux environments.**

### Hardware
- A PC running Windows or Linux
- A [Microsoft Azure Kinect Camera](https://www.microsoft.com/en-us/d/azure-kinect-dk/8pp5vxmd9nhq?activetab=pivot:overviewtab)
- Optional: A Robovie robot! You can also just run the simulator, or can adjust the angle constraints in the scripts to use your own robot!

### Software
#### Step 1: Clone the RoboMime repository
- Clone this repo by running:

    ```git clone https://github.com/JoJoFence/RoboMime.git```

#### Step 2: Python packages via pip
- Next you will need to install some packages via pip. In the RoboMime repo you just cloned, run:
  
    ```pip install -r requirements.txt```
  
  - The following packages will be installed:
    - pykinect_azure
    - numpy
    - OpenCV
    - keyboard
    - scipy
   
#### Step 3: Python, Visual Studio, and Azure Kinect SDKs
- [Python 3.x](https://www.python.org/downloads/)
  - **Note:** Make sure the Python path is added to your PATH environment variable! 
- [Visual Studio](https://visualstudio.microsoft.com/downloads/)
  - **Note:** Make sure the ```C:\Users\[USERNAME]\AppData\Local\Programs\Microsoft VS Code\bin``` path is added to your PATH environment variable!
- [Microsoft Azure Kinect Sensor SDK](https://learn.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download)
  - **Note:** Make sure the ```C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\bin\depthengine_2_0.dll``` and ```C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\bin\k4a.dll``` paths are added to you PATH environment variable!
- [Microsoft Azure Kinect Body Tracking SDK](https://learn.microsoft.com/en-us/azure/kinect-dk/body-sdk-download)
  - **Note:** Make sure the ```C:\Program Files\Azure Kinect Body Tracking SDK\tools``` path is added to your PATH environment variable.



  
## Running the Simulator

#### Step 1
- Plug your Azure Kinect camera into your PC and into power, point the camera towards you, and boot up the simulator by going into the ```MotionVisualizer``` folder of the repo and double-clicking on the "MotionVisualizer" application.

#### Step 2
- With the MotionVisualizer simulator running, navigate to the ```Python_Scripts``` folder of the repo and execute the following command:

    ```python send_angles_to_robovie.py -s -c```

  - **Note:** The ```-s``` and ```-c``` options tell the Python script to send data to the simulator (```-s```) and to launch the Azure Kinect camera view (```-c```). To see all options, you can run the following:

      ```python send_angles_to_robovie.py --help``` 

#### The simulator in action:
https://github.com/JoJoFence/RoboMime/assets/26200490/35561fd7-031f-4916-ac87-831ad0d86789



## Running on the Robovie Robot

#### Step 1
- Plug your Azure Kinect camera into your PC and into power, point the camera towards you, and boot up your robot.

#### Step 2
- Once your robot is turned on, connect your PC to its Wifi network.

#### Step 4
- Run the following command:

    ```python send_angles_to_robovie.py -c -a [YOUR_ROBOT'S_IP_ADDRESS]```
  
  - **Note:** Be sure to replace ```[YOUR_ROBOT'S_IP_ADDRESS]``` with the IP address of your robot. The ```-c``` option launches the Azure Kinect camera view.
 
#### Step 5
- **Mandatory:** Have fun!

#### Testing on a Robovie robot:
https://github.com/JoJoFence/RoboMime/assets/26200490/9ec97350-de4e-4817-a63a-c4c0869cf56c

