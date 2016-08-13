include		../../Makefile.rules

# Source files
OBJS			:= bytecach.o lock.o memblck.o persist.o resource.o stmcopy.o \
					stmfs.o stmmem.o stmprbin.o stmprxml.o stmsrc.o stmsrcfs.o stream.o

# Library
all						:	$(OUTLOC)libiol.a

$(OUTLOC)libiol.a		:	$(OBJS)
								$(AR) r $(OUTLOC)libiol.a $(OBJS)

clean						:
								rm -f *.o
								rm -f $(OUTLOC)libiol.a

