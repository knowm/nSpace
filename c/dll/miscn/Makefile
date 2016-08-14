include		../../Makefile.rules

# Source files
OBJS		:= miscn.o
LIBS		:= -lmiscl -lsysl -lnspcl -ladtl -lccl
LDFLAGS		:= $(LDFLAGS) -L$(OUTLOC) # -Wl,--versmiscn-script=../../lib/ccl/ccl.version

# Library
all						:	$(OUTLOC)libmiscn.so

$(OUTLOC)libmiscn.so	:	$(OBJS)
									$(LD) $(LDFLAGS) -o $(OUTLOC)libmiscn.so $(OBJS) $(LIBS)

clean						:
									rm -f *.o
									rm -f $(OUTLOC)libmiscn.so

