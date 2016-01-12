include		../../rules.unix

# Source files
OBJS			:= asynce.o asyncq.o clone.o compare.o create.o dctfmt.o dctprs.o debug.o \
				decode.o demux.o dist.o strfmt.o string.o strprs.o strstm.o \
				time.o timer.o toggle.o tokenit.o type.o

# Library
all						:	$(OUTLOC)libmiscl.a

$(OUTLOC)libmiscl.a	:	$(OBJS)
								$(AR) r $(OUTLOC)libmiscl.a $(OBJS)

clean						:
								rm -f *.o
								rm -f $(OUTLOC)libmiscl.a

