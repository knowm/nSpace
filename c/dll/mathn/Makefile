include		../../Makefile.rules

# Source files
OBJS	:= mathn.o
LIBS	:= -lmathl -lnspcl -ladtl -lccl -lsysl
LDFLAGS	:= $(LDFLAGS) -L$(OUTLOC) -Wl,--version-script=../../lib/ccl/ccl.version

# Library
all							:	$(OUTLOC)libmathn.so

$(OUTLOC)libmathn.so	:	$(OBJS)
									$(LD) $(LDFLAGS) -o $(OUTLOC)libmathn.so $(OBJS) $(LIBS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)libmathn.so

