CXX           = g++
DEFINES       = -DQT_NO_DEBUG -DQT_WIDGETS_LIB -DQT_GUI_LIB -DQT_CORE_LIB
CFLAGS        = -pipe -O3 -Wall -fexceptions -pthread -O2 -Wall -W -D_REENTRANT -fPIC $(DEFINES)
CXXFLAGS      = -pipe -O2 -Wall -W -D_REENTRANT -fPIC $(DEFINES)
INCPATH       = -I. -isystem /usr/local/include/opencv4 
LIBS          = -lopencv_gapi -lz -lopencv_stitching -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_cvv -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_highgui -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_dnn -lopencv_plot -lopencv_videostab -lopencv_videoio -lopencv_xfeatures2d -lopencv_shape -lopencv_ml -lopencv_ximgproc -lopencv_video -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core



SEASOCKS := ./seasocks

INCLUDES := -I $(SEASOCKS) -I $(SEASOCKS)/src/main/c/internal -I $(SEASOCKS)/src/main/c/seasocks \
	-I $(SEASOCKS)/src/main/c -I $(SEASOCKS)/build/src/main/c \
	-I. -isystem /usr/local/include/opencv4 
	
LDFLAGS += -L$(SEASOCKS)/build/src/main/cs

LINKLIBS := $(SEASOCKS)/build/src/main/c/libseasocks.a
	
all: server

server: server.cpp
	$(CXX) $(INCLUDES) server.cpp $(LIBS) $(LDFLAGS) -o server
cv2:cv2.cpp base64.o
	$(CXX) $(INCLUDES) base64.o cv2.cpp $(LIBS) $(LINKLIBS) $(LDFLAGS) -o cv2 
base64.o: base64.cpp base64.h
	$(CXX) $(INCLUDES) base64.cpp $(LIBS) $(LDFLAGS) -c
clean:
	rm -rf *.o *.out