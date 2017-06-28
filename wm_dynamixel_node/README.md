# wm_dynamixel_hardware_interface

Package pour le hardware interface des moteurs dynamixel de sara.
Ce package agit comme un plugin pour sara_control. Par conséquent, aucun launch direct n'est néssésaire.
Pour tester ce package:
```
git clone https://github.com/WalkingMachine/sara_launch.git
git clone https://github.com/WalkingMachine/sara_control.git
git clone https://github.com/WalkingMachine/sara_description.git
git clone https://github.com/WalkingMachine/dynamixel_c2_description.git
```
Install
```
sudo apt-get install ros-kinetic-ros-control
catkin_make
```
Launch
```
roslaunch sara_description sara_description
roslaunch sara_control sara_control
roslaunch sara_launch sara_control
```

les controleur sara_arm_trajectory_controller et sara_arm_velocity_controller devrais alors être disponnible pour ere loader avec la commande $rosservice call /controller_manager/load_controller "name: '<le_nom_du_controlleur>'"
  
