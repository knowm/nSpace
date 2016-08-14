include		../../Makefile.rules

# Source files
OBJS	:= netn.o
LIBS	:= -lnetl -lnspcl -ladtl -lccl -lsysl
LDFLAGS	:= $(LDFLAGS) -L$(OUTLOC) -Wl,--version-script=../../lib/ccl/ccl.version

# Library
all							:	$(OUTLOC)libnetn.so

$(OUTLOC)libnetn.so		:	$(OBJS)
									$(LD) $(LDFLAGS) -o $(OUTLOC)libnetn.so $(OBJS) $(LIBS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)libnetn.so

