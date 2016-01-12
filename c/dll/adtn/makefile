include		../../rules.unix

# Source files
OBJS			:= adtn.o
LIBS			:= -lccl -ladtl -lsysl
LDFLAGS		:= $(LDFLAGS) -L$(OUTLOC)

# Library
all							:	$(OUTLOC)libadtn.so

$(OUTLOC)libadtn.so		:	$(OBJS)
									$(LD) $(LDFLAGS) -o $(OUTLOC)libadtn.so $(OBJS) $(LIBS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)libadtn.so

