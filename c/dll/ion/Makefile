include		../../Makefile.rules

# Source files
OBJS	:= ion.o
LIBS	:= -liol -lnspcl -ladtl -lsysl -lccl
LDFLAGS	:= $(LDFLAGS) -L$(OUTLOC) -Wl,--version-script=../../lib/ccl/ccl.version

# Library
all							:	$(OUTLOC)libion.so

$(OUTLOC)libion.so	:	$(OBJS)
									$(LD) $(LDFLAGS) -o $(OUTLOC)libion.so $(OBJS) $(LIBS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)libion.so

