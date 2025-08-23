## Instructions 
  1. Connect Pockey's Wifi
  2. Remote ssh using <br> ```ssh pi@192.168.17.1```
  3. Get into bash_file directory on two terminals
  4. Run ```./00_launch.sh``` (to initiate the robot) and ```./07_run.sh``` (to run a code file) each terminal
  5. If you want to edit/ update a file. You need to stop a running code, correct your code
in VScode and then open your device's terminal and get in the directory where the written file exist. After that,
```scp turtlebot3_controller.py pi@192.168.17.1:/home/pi/turtlebot3_ws/src/turtlebot3_controller/turtlebot3_controller```
  7. Don't forget to ```crtl+c``` and poweroff raspberry pi before turn off the robot to prevent damages on the hardware.
