all: src/std_msgs src/common_msgs .catkin_tools
	ROS_LANG_DISABLE=geneus:genlisp:gennodejs:genpy catkin build

clean:
	catkin clean

rebuild: clean all

.catkin_tools:
	catkin init

src/std_msgs:
	git submodule update --init

src/common_msgs:
	git submodule update --init

.PHONY: all clean rebuild
