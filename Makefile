CC=g++
CFLAGS=-c -O3
LDFLAGS =-lGL -lGLU -lglut
SOURCES=as4.cpp Bone.cpp Kinematics.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=as4

RM = /bin/rm -f

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	$(RM) *.o $(EXECUTABLE)
