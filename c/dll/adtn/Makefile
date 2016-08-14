include		../../Makefile.rules

# Source files
OBJS			:= adtn.o
LIBS			:= -lccl -ladtl -lsysl -lnspcl
LDFLAGS		:= $(LDFLAGS) -L$(OUTLOC) -Wl,--version-script=../../lib/ccl/ccl.version

# Library
all							:	$(OUTLOC)libadtn.so

$(OUTLOC)libadtn.so		:	$(OBJS)
									$(LD) $(LDFLAGS) -o $(OUTLOC)libadtn.so $(OBJS) $(LIBS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)libadtn.so

