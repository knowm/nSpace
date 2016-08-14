include		../../Makefile.rules

# Source files
OBJS			:= 	binary.o counter.o datablk.o formula.o mat3d.o mathl.o \
				trans3d.o unary.o

# Library
all						:	$(OUTLOC)libmathl.a

$(OUTLOC)libmathl.a	:	$(OBJS)
								$(AR) r $(OUTLOC)libmathl.a $(OBJS)

clean						:
								rm -f *.o
								rm -f $(OUTLOC)libmathl.a

