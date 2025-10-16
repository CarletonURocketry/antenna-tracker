CC = gcc
CFLAGS = -Wall -Wextra -DDESKTOP_BUILD
OUT = tracker

SRCDIR = $(abspath ./src)
SRCS = $(shell find $(SRCDIR) -name "*.c")

OBJS = $(patsubst %.c,%.o,$(SRCS))

all: $(OUT)

$(OUT): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(OUT)

%.o: %.c
	$(CC) $(CFLAGS) $(WARNINGS) -o $@ -c $<

clean:
	@rm $(OUT)
	@rm $(OBJS)
