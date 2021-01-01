SRC_DIR = $(CURDIR)/src
all: 
	g++ -g -std=c++11 -I. -I../rapidjson/include/ -Iheaders/ -I../rr-v2x-common/ RaceTest.cc SimMove.cc MathVector.cc -o test
