## Modifications to msg and srv

### No variable-size arrays
- all array storage (greater than max size) separate from serializable type

### ROS time & duration
- Fw::Time with specific timebase/context

### string

### Header

## Build system

### Requirements
- RTTI??
- how to disable RTTI everywhere with no ROS includes

### Serializable types and Ports
- version-control?

### methods for converting to and from ROS message objects
- F Prime templates and visitors to create converter code to/from ROS message objects?

### Unit tests
- headers
- link time

## ROS system integration

### roslaunch/rosrun
- stub package for each topology
- how to register fprime-built executable with catkin/ROS?

### pub/sub
- full=drop on ports?
