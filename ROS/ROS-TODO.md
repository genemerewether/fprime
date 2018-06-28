## ROS Components

### RosCycle

- provide reset command to set time back to zero?
    - or should we just shutdown - there is accumulated state throughout the application

## Modifications to msg and srv

### No variable-size arrays
- all array storage (greater than max size) separate from serializable type

## Build system

### Requirements
- RTTI??
- how to disable RTTI everywhere with no ROS includes

### methods for converting to and from ROS message objects
- F Prime templates and visitors to create converter code to/from ROS message objects?
- or python generate from msg spec

### Unit tests
- headers
- link time

## ROS system integration

### roslaunch/rosrun
- stub package for each topology
- how to register fprime-built executable with catkin/ROS?

### pub/sub
- full=drop on ports?
