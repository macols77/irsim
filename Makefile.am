#INCLUDES = -Ifitnessfunctions -Iexperiments -Isensors -Irenders -Irenders/drawstuff -Icollisions -Iobjects -Icontrollers -Iactuators -Iarenas -Ipopulations -I/usr/X11R6/include -I/usr/include/gsl
INCLUDES = -Iexperiments -Isensors -Irenders -Irenders/drawstuff -Icollisions -Iobjects -Icontrollers -Iactuators -Iarenas -Ipopulations -I fitnessfunctions -I/usr/X11R6/include -I/usr/include/gsl

#SUBDIRS = collisions renders arenas fitnessfunctions sensors controllers \
#	  actuators populations experiments objects

#SUBDIRS = objects sensors controllers actuators arenas populations experiments collisions fitnessfunctions renders
SUBDIRS = objects sensors controllers actuators arenas experiments collisions renders populations fitnessfunctions

bin_PROGRAMS  = irsim
irsim_SOURCES = experiment.cpp             \
		 random.cpp				                \
		 main.cpp                               \
		 arena.cpp                              \
		 render.cpp                             \
		 simulator.cpp                          \
		 sensor.cpp                             \
		 actuator.cpp                           \
		 controller.cpp                         \
		 geometry.cpp                           \
		 epuck.cpp                              \
		 puck.cpp                               \
		 simmath.cpp				            \
		 arguments.cpp                          \
		 simobject.cpp													\
		 general.cpp														\
		 population.cpp													\
		 individual.cpp												  \
		 fitnessfunction.cpp

irsim_LDADD   = experiments/libexperiments.a \
									 objects/libobjects.a \
									 arenas/libarenas.a \
									 actuators/libactuators.a \
									 sensors/libsensors.a \
									 controllers/libcontrollers.a \
									 collisions/libcollisions.a \
									 renders/librenders.a \
									 renders/drawstuff/libdrawstuff.a \
									 populations/libpopulations.a \
									 fitnessfunctions/libfitnessfunctions.a \
									 -lpthread -lGL -lGLU -lX11 -lgsl -lgslcblas -L/usr/X11R6/lib 
