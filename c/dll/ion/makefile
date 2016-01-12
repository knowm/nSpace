include		../../rules.unix

# Source files
OBJS			:= ion.o
LIBS			:= -liol -lccl -ladtl -lsysl
LDFLAGS		:= $(LDFLAGS) -L$(OUTLOC)

# Library
all							:	$(OUTLOC)libion.so

$(OUTLOC)libion.so	:	$(OBJS)
									$(LD) $(LDFLAGS) -o $(OUTLOC)libion.so $(OBJS) $(LIBS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)libion.so

