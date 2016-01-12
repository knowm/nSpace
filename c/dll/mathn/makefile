include		../../rules.unix

# Source files
OBJS			:= mathn.o
LIBS			:= -lmathl -lccl -ladtl -lsysl
LDFLAGS		:= $(LDFLAGS) -L$(OUTLOC) # -Wl,--versmathn-script=../../lib/ccl/ccl.version

# Library
all							:	$(OUTLOC)libmathn.so

$(OUTLOC)libmathn.so	:	$(OBJS)
									$(LD) $(LDFLAGS) -o $(OUTLOC)libmathn.so $(OBJS) $(LIBS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)libmathn.so

