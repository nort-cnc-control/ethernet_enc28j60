SRCS :=	src/control.c	\
	src/eth_codec.c	\
	src/init.c	\
	src/registers.c	\
	src/rx.c	\
	src/tx.c

CC += -I$(shell pwd)/include

OBJS := $(SRCS:%.c=%.o)
TARGET := libenc28j60.a

%.o : %.c
	$(CC) -c $< -o $@

$(TARGET): $(OBJS)
	$(AR) rsc $@ $^

clean:
	rm -f $(OBJS) $(TARGET)

