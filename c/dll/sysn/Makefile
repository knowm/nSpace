include		../../Makefile.rules

# Source files
OBJS			:= sysn.o
LIBS			:= -lsysl -lnspcl -lccl -ladtl -lsysl
ifeq ($(OS),Linux)
#LIBS			:= $(LIBS) -luuid
endif
LDFLAGS			:= $(LDFLAGS) -L$(OUTLOC) # -Wl,--verssysn-script=../../lib/ccl/ccl.version

# Library
all							:	$(OUTLOC)/libsysn.so

$(OUTLOC)/libsysn.so	:	$(OBJS)
								$(LD) $(LDFLAGS) -o $(OUTLOC)/libsysn.so $(OBJS) $(LIBS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)/libsysn.so

