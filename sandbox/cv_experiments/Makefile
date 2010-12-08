
all:
#	g++ -o filters filters.cpp -I/usr/include/opencv -L/usr/lib -lcv -lcxcore -lcvaux -lhighgui
	g++ -o filters filters.cpp -I/opt/ros/cturtle/stacks/vision_opencv/opencv2/opencv/include/opencv -I/opt/ros/cturtle/stacks/vision_opencv/opencv2/opencv/include -L/opt/ros/cturtle/stacks/vision_opencv/opencv2/opencv/lib -Wl,-rpath,/opt/ros/cturtle/stacks/vision_opencv/opencv2/opencv/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_objdetect -lopencv_features2d -lopencv_calib3d -lopencv_contrib -lopencv_legacy
	
clean:
	rm -f test
	rm -f two



