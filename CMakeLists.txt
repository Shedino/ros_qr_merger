cmake_minimum_required(VERSION 2.8.3)

project(ros_qr_merger)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs nav_msgs message_generation ros_qr_tracker)

catkin_python_setup()

#add_message_files(FILES
#    Percept.msg
#)

add_service_files(FILES
    GetTargetPosition.srv
    SaveData.srv
    LoadData.srv
)

generate_messages(   
    DEPENDENCIES  
    std_msgs  
    geometry_msgs  
    nav_msgs  
)  

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

#############
## Install ##
#############

install(PROGRAMS
     nodes/qr_merger.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add folders to be run by python nosetests

