CC ?= gcc
CFLAGS ?= -MMD -MP -I.. -I.
CFLAGS += -g
CFLAGS += -Werror -Wall -Wextra -Wno-unused-variable -Wno-unused-value

ifneq ($(V), 1)
Q = @
endif