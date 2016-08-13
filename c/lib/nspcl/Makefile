include		../../Makefile.rules

# Source files
OBJS		:= connlst.o conns.o glbnspc.o keypath.o \
			link.o loc.o namespc.o nspcl.o parser.o \
			reflect.o tempimpl.o temploc.o temporal.o this.o value.o

# Library
all							:	$(OUTLOC)libnspcl.a

$(OUTLOC)libnspcl.a		:	$(OBJS)
									$(AR) r $(OUTLOC)libnspcl.a $(OBJS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)libnspcl.a

