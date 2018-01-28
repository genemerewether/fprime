## Modifications to msg and srv

### No variable-size arrays
- all array storage separate from serializable type

### ROS time & duration
- Fw::Time with specific timebase/context

### string

### Header

## Build system

### Serializable types
- generate Ai.xml from .msg
  - manually add serializable Ai.xml to port Ai.xml
  - dependency graph?
- version-control - these types will be custom so should not have any dependencies on the user's ROS paths
- put them in fprime/ROS/Types with *_msgs directory structure

### methods for converting to and from ROS message objects
- 

### Unit tests
- headers
- link time

## ROS system integration

### roslaunch/rosrun
- stub package for each topology
- how to register fprime-built executable with catkin/ROS?

### pub/sub
- full=drop on ports?


## misc notes
- multiple components may need same (non-standard) messages
  - can't have duplicate defs in 2 catkin workspaces
  - ROS/Types/*_msgs
    - catkin build step should put SerializableAi.xml files here
    - can check in to repo if desired
  - ROS/fprime_ws/src
    - can add custom msg packages here
    - also has genfprime
      - Validate msg against Autocoders/schema/default/serializable_schema.rng
      - Validate srv against Autocoders/schema/default/interface_schema.rng
    - genfprime
      - generate_messages(DEPENDENCIES deps [LANGS lang1 lang2...])
- generate port from msg/srv
  - msg port has no return value; srv port does
