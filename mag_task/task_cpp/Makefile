# building project in GCC / Linux

LIBS = -lpython2.7 -lopencv_core -lopencv_highgui

ur_main: ur_main.cpp py_bind.cpp
	g++ -Wall ur_main.cpp py_bind.cpp -o ur_main $(LIBS)

