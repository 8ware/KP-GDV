NAME := vision
SRC  := $(NAME).cpp
BIN  := $(NAME)

GCC   = g++
FLAGS = `pkg-config --cflags libopenni opencv`
LIBS  = `pkg-config --libs libopenni opencv`

.PHONY: all $(NAME)
all: $(NAME)

$(NAME):
	@$(GCC) $(FLAGS) $(LIBS) -o $(BIN) $(SRC)

