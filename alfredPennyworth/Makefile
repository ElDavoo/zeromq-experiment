CC = gcc
CFLAGS = -Wall -Wextra
FLAGS = -lczmq -lzmq

SRCS := broker.c cliente.c worker.c
EXES := $(SRCS:.c=)
LIB_SRCS := cJSON.c
LIB_OBJ := $(LIB_SRCS:.c=.o)
LIB_NAME := libjson_utils.a

.PHONY: all clean

all: $(EXES)

%: %.c $(LIB_NAME)
	$(CC) -o $@ $< $(LIB_NAME) $(FLAGS)

$(LIB_NAME): $(LIB_OBJ)
	ar rcs $(LIB_NAME) $(LIB_OBJ)

clean:
	rm -f $(EXES) $(LIB_NAME) $(LIB_OBJ)