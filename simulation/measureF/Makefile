.SILENT:

LIBRARIES = -llapack -lGL -lglut -lGLU -Ofast
LIB_PATH = /usr/local/include/eigen3
STD = -std=c++11

default: clean simDER

simDER:  main.cpp world.cpp elasticRod.cpp elasticStretchingForce.cpp elasticBendingForce.cpp elasticTwistingForce.cpp externalGravityForce.cpp inertialForce.cpp dampingForce.cpp timeStepper.cpp setInput.cpp
	g++ -O2 -Wno-deprecated-declarations $(STD) -I $(LIB_PATH) main.cpp world.cpp elasticRod.cpp elasticStretchingForce.cpp elasticBendingForce.cpp elasticTwistingForce.cpp externalGravityForce.cpp inertialForce.cpp dampingForce.cpp timeStepper.cpp setInput.cpp  $(LIBRARIES) -o simDER

clean:
	rm -rf simDER

run:
	./simDER option.txt
