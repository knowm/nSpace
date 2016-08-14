include		../../Makefile.rules

# Source files
OBJS	:= sysn.o
LIBS	:= -lnspcl -ladtl -lccl -lsysl
LDFLAGS	:= $(LDFLAGS) -L$(OUTLOC) -Wl,--version-script=../../lib/ccl/ccl.version

# Library
all							:	$(OUTLOC)/libsysn.so

$(OUTLOC)/libsysn.so	:	$(OBJS)
								$(LD) $(LDFLAGS) -o $(OUTLOC)/libsysn.so $(OBJS) $(LIBS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)/libsysn.so

