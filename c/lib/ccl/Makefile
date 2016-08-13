include		../../Makefile.rules

# Source files
OBJS			:= factory.o object.o ccl.o register.o
IMPLOBJS		:= ccl_impl.o

# Library
all							:	$(OUTLOC)libccl.a $(OUTLOC)libccl_impl.a

$(OUTLOC)libccl.a			:	$(OBJS)
									$(AR) r $(OUTLOC)libccl.a $(OBJS)

$(OUTLOC)libccl_impl.a	:	$(IMPLOBJS)
									$(AR) r $(OUTLOC)libccl_impl.a $(IMPLOBJS)


clean							:
									rm -f *.o
									rm -f $(OUTLOC)libccl.a
									rm -f $(OUTLOC)libccl_impl.a

