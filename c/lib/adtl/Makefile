include		../../Makefile.rules

# Source files
OBJS			:= avl.o bool.o date.o double.o float.o int.o long.o string.o iunknown.o value.o \
					dict.o iterate.o keys.o list.o load.o queue.o remove.o stack.o stat.o store.o write.o

# Library
all							:	$(OUTLOC)libadtl.a

$(OUTLOC)libadtl.a		:	$(OBJS)
									$(AR) r $(OUTLOC)libadtl.a $(OBJS)

clean							:
									rm -f *.o
									rm -f $(OUTLOC)libadtl.a

