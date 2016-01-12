include		../../rules.unix

# Source files
OBJS			:= address.o avail.o client.o dgram.o intfs.o multicast.o netl.o persist.o \
					recv.o send.o socket.o
					
# Library
all						:	$(OUTLOC)libnetl.a

$(OUTLOC)libnetl.a	:	$(OBJS)
								$(AR) r $(OUTLOC)libnetl.a $(OBJS)

clean						:
								rm -f *.o
								rm -f $(OUTLOC)libnetl.a

