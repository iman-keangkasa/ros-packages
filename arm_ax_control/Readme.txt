## Execution
To launch the controller for dynamixel ax12+ arm, run the ros launch file:

```
roslaunch arm_ax_control moveit_controllers_and_robot.launch 
```

It requires the installation of ros packages:

- moveit
- dynamixel_sdk
- dynamixel_workbench
- joy  (only if the controller is required)

It is necessary to connect dynamixel motors through usb2dynamixel adapter.
To give permisions to the usb adapter run:
```
sudo chmod a+rw /dev/ttyUSB0
```

With your correct device name, ex. dev/ttyUSB1, etc.



