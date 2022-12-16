# ROS 2 configuration and launch files for Neobotix MP-500

This package contains configuration and launch files for Neobotix MP-500.

![Neobotix MP-500](https://www.neobotix-robots.com/products/mobile-robots/mobile-robot-mp-500)

The [MP-500](https://www.neobotix-robots.com/products/mobile-robots/mobile-robot-mp-500) is a small and robust robot for research and industries.

# Documentation

This branch has a special configuration for switching the scan fields. This feature is taken care by the `configure_relays` node. There are three different scan fields reached at 3 different speeds, which are basically toggled by the relays. Scanner fields are pre-set to the scanner.

The 3 different speeds can be configured under  `neo_mp_500-2/configs/relayboard_v2/config_relays.yaml`. 

Please find the rest of our documentations at https://neobotix-docs.de/ros/ros2/index.html

# Contact information

For more information please visit our website at www.neobotix-robots.com. 
If you have any questions, just get in touch with us:
* General information: http://www.neobotix-robots.com/company-contact.html
* ROS related questions: ros@neobotix.de
