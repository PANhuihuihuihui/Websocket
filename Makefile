CXX           = g++
DEFINES       = -DQT_NO_DEBUG -DQT_WIDGETS_LIB -DQT_GUI_LIB -DQT_CORE_LIB
CFLAGS        = -pipe -O3 -Wall -fexceptions -pthread -O2 -Wall -W -D_REENTRANT -fPIC $(DEFINES)
CXXFLAGS      = -pipe -O2 -Wall -W -D_REENTRANT -fPIC $(DEFINES)
INCPATH       = -I. -isystem /usr/local/include/opencv4 




SEASOCKS := ./seasocks

INCLUDES := -I $(SEASOCKS) -I $(SEASOCKS)/src/main/c/internal -I $(SEASOCKS)/src/main/c/seasocks \
	-I $(SEASOCKS)/src/main/c -I $(SEASOCKS)/build/src/main/c \
	-I /usr/local/include/opencv4 

LIBS:= -lseasocks
	
LDFLAGS += -L$(SEASOCKS)/build/src/main/c

LINKLIBS := $(SEASOCKS)/build/src/main/c/libseasocks.so.1.4.3
	
all: server

serverws: server.cpp
	$(CXX) $(INCLUDES) server.cpp $(LIBS) $(LDFLAGS) -o server

clean:
	rm -rf *.o *.out