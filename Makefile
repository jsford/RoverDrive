.SILENT:

.PHONY: all clean

all:
	catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -j12

clean:
	rm -rf build/ devel/
