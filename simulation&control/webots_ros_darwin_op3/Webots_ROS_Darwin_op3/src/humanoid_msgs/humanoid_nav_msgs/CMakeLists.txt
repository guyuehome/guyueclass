cmake_minimum_required(VERSION 2.8.3)
project(humanoid_nav_msgs)

#List to make rest of code more readable
set( MESSAGE_DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

#Declare build dependencies
find_package(catkin REQUIRED
    COMPONENTS
    message_generation
    ${MESSAGE_DEPENDENCIES} )

#Add message files
add_message_files(DIRECTORY msg FILES StepTarget.msg StepPlan.msg)

#Add service files
add_service_files(DIRECTORY srv
    FILES
    ClipFootstep.srv
    PlanFootsteps.srv
    PlanFootstepsBetweenFeet.srv
    StepTargetService.srv )

#Add action files
add_action_files(DIRECTORY action
    FILES
    ExecFootsteps.action)

#And now generate the messages
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

# Generate catkin/pkg-config import information
catkin_package( CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES} )

