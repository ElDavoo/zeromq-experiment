CC = gcc
# CFLAGS = -Wall -Wextra -I../../../include
FLAGS = -lczmq -lzmq -I../../../common

SRCS := broker.c cliente.c worker.c
EXES := $(SRCS:.c=)
LIB_SRCS := cJSON.c
LIB_OBJ := $(LIB_SRCS:.c=.o)
LIB_NAME := libjson_utils.a
COMMON_DIR := ../../../common

# Imposta il percorso relativo alla directory "common" rispetto al Makefile
vpath %.c ../../../common

.PHONY: all clean

all: $(EXES)

%: %.c $(LIB_NAME)
	$(CC) -o $@ $< $(COMMON_DIR)/$(LIB_NAME) $(FLAGS)

$(LIB_NAME): $(LIB_OBJ)
	ar rcs $(COMMON_DIR)/$(LIB_NAME) $(LIB_OBJ)

clean:
	rm -f $(EXES) $(COMMON_DIR)/$(LIB_NAME) $(LIB_OBJ)