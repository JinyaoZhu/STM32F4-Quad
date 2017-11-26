# STM32F4-Quad
An STM32F4 based Quadrotor Flight Controller running on the RTOS uC/OS-III, which mainly tested in the indoor environment. Including attitude control, position control and trajectory tracking.

A simple vision-based indoor navigation system is also provided, which using active infrared makers as the locators.

The Flight Controller is developed in IDE uVison Keil4 using C language, and the indoor positioning system is based on OpenCV library.

## Dissertation  
https://github.com/JinyaoZhu/STM32F4-Quad/blob/master/dissertation.pdf

## Folder  
./Indoor_Positionning_System : source code for indoor positionning system in PC\
./Matlab : Matlab simulation of the quadrotor\
./Quad_indoor_cam(vision) : source code of the flight controller\
./Quad_3D_display : position and attitude visualization\
./Stereo Camera Calibration : code for camera calibration\
./tex : source code for the dissertation

## Test video
**Playing music:**  
https://youtu.be/FTTgFP2V9RU "Little star"

**Gesture control:**  
https://youtu.be/_VyHRGHKnpY with kinect

**Trajectory tracking:**
* 1: https://youtu.be/Airv29XN67Q curve1
* 2: https://youtu.be/eWLOpIESicU 1D sine
* 3: https://youtu.be/p_XNRUGR_co 3D 
* 4: https://youtu.be/EVki9DBirWQ straight line
* 5: https://youtu.be/JW-OWvRWwpA planar rotate
