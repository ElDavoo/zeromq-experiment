CC = gcc
CFLAGS = -Wall -Wextra
FLAGS = -lzmq

SRCS := $(wildcard *.c)
EXES := $(SRCS:.c=)

.PHONY: all clean

all: $(EXES)

%: %.c
	$(CC) -o $@ $< $(FLAGS)

clean:
	rm -f $(EXES)