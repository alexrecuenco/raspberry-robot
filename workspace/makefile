# https://stackoverflow.com/a/28560379
OUT      = main
TEST     = test
CALIBRATE= calibrate


SRCFILES := $(wildcard *.c) $(wildcard **/*.c)

OBJFILES = $(SRCFILES:.c=.o)
DEPFILES = $(OBJFILES:.o=.d)
# disabled warnings
# -Wmissing-prototypes -Wmissing-declarations
WARNINGS := -Wall -Wextra -pedantic -Wshadow -Wpointer-arith -Wcast-align \
            -Wwrite-strings \
            -Wredundant-decls -Wnested-externs -Winline \
            -Wuninitialized -Wconversion -Wstrict-prototypes

#DEBUG	= -g -O0
#  -std=c11 Maybe add this?
DEBUG ?= -g -O3
CC	?= gcc
INCLUDE	= -I. -I/usr/local/include
CFLAGS	= -std=c11 -g $(WARNINGS) $(DEBUG) -pipe $(EXTRA_CFLAGS)
ALL_CFLAGS = $(INCLUDE) $(CFLAGS)  -MMD -MP
LDFLAGS	= -L/usr/local/lib
LDLIBS    = -lpigpio -lwiringPi -lwiringPiDev -lpthread -lm -lcrypt -lrt

.PHONY: all clean

all: main calibrate tests speeds

clean:
	-@$(RM) $(wildcard $(OBJFILES) $(DEPFILES) $(PROJNAME))

-include $(DEPFILES)

# The @ makes sure these commands are not printed
%.o: %.c
	@$(CC) $(ALL_CFLAGS) -c -o $@ $<

# TODO: Make this more generic, rather than just remove .o files manually
main: $(filter-out src/calibrate.o test/test.o src/speeds.o, $(OBJFILES))
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

calibrate: $(filter-out src/main.o test/test.o src/speeds.o, $(OBJFILES))
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

tests: $(filter-out src/main.o src/calibrate.o src/speeds.o, $(OBJFILES))
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

speeds: $(filter-out src/main.o src/calibrate.o test/test.o, $(OBJFILES))
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)
