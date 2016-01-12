include		../../rules.unix

# Source files
OBJS			:=	main.o
ifeq ($(OS),Linux)
LDFLAGS		:= -L$(OUTLOC) -Wl,--version-script=nsh.version -export-dynamic 
else
LDFLAGS		:= -L$(OUTLOC) -L/usr/local/lib -lpthread
endif
# -lpthread 
# -Wl,--version-script=nsh.version -export-dynamic 
#LIBS			:= -luil -lccl_impl -ladtl -lccl -lsysl -lwin32l
LIBS			:= -lnshl -lccl_impl -ladtl -lccl -lsysl 
#LIBS			:= -ladtl  -lccl_impl -lccl -lsysl
#CFLAGS		+= $(CFLAGS)

# -lpthread

# Application
all					:	$(OUTLOC)nsh

$(OUTLOC)nsh		:	$(OBJS)
							$(CC) $(LDFLAGS) -o $(OUTLOC)nsh $(OBJS) $(LIBS) -ldl 

clean						:
								rm -f *.o
								rm -f $(OUTLOC)nsh

