include		../../rules.unix

# Source files
OBJS			:= cs.o dbg.o dl.o event.o log.o memory.o string.o thread.o utiluuid.o

# Library
all							:	$(OUTLOC)libsysl.a

$(OUTLOC)libsysl.a		:	$(OBJS)
									$(AR) r $(OUTLOC)libsysl.a $(OBJS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)libsysl.a

