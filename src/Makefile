CXX		=	g++
CXXFLAGS	=	-g -O3 -Wall -static-libstdc++

all:		predict

predict:	predict.cc trace.cc predictor.h branch.h trace.h my_predictor.h tage.h loop_predictor.h
		$(CXX) $(CXXFLAGS) -o predict predict.cc trace.cc

clean:
		rm -f predict
