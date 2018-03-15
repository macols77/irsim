
DEBUG	= 1
CC 	= g++
ifeq ($(DEBUG),1)
	DEBFLAGS = -g -O -DDEBUG 
else
	DEBFLAGS = -O2
endif

CFLAGS = -DLINUX $(DEBFLAGS) 
# need also thread support for interrupt handling
#LFLAGS += -lpthread
# si se have con gcc en vez de g++: LFLAGS  +=   -lm -Wall -lstdc++
# enves de -lm es más amplio -lglkp
LFLAGS  +=   -lm -Wall 
#LFLAGS  +=   -lm -Wall $(pkg-config --cflags --libs comedilib)
#LFLAGS  =  -lpthread -lm -nostdlib -lgcc



TARGET = irsimGnuplot
SRCS =   plotter.cpp   

OD = ./
OBJS = $(addsuffix .o, $(addprefix $(OD), $(basename $(notdir $(SRCS)))))

$(TARGET) : $(OBJS)
	$(CC) $(LFLAGS) -o $@ $(OBJS) 
plotter.o : ./plotter.cpp
	$(CC) -c $(CFLAGS) -o $@ $< 

all : $(TARGET)

clean :
	rm -f *~ $(OBJS) $(TARGET) 
