CXX=g++
CXXFLAGS = -I/usr/local/include/libfreenect -I/usr/local/include/opencv 




LINKFLAGS = -L/usr/local/lib -lopencv_core -lopencv_features2d -lopencv_highgui -lopencv_legacy -lopencv_imgproc -lfreenect
SAMPLES_C = $(notdir $(patsubst %.c, %, $(wildcard *.c)))
SAMPLES_CPP = $(notdir $(patsubst %.cpp, %, $(wildcard *.cpp)))

SAMPLES = $(SAMPLES_C) $(SAMPLES_CPP)

all: $(SAMPLES)

%: %.c
	@echo $@
	@$(CXX) $(CXXFLAGS) -o $@ $< $(LINKFLAGS) 

%: %.cpp
	@echo $@
	@$(CXX) $(CXXFLAGS) -o $@ $< $(LINKFLAGS) 
