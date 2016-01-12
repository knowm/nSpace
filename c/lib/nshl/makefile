include		../../rules.unix

# Source files
OBJS			:= maindef.o nshl.o shell.o
					
# Library
all						:	$(OUTLOC)libnshl.a

$(OUTLOC)libnshl.a	:	$(OBJS)
								$(AR) r $(OUTLOC)libnshl.a $(OBJS)

clean						:
								rm -f *.o
								rm -f $(OUTLOC)libnshl.a

