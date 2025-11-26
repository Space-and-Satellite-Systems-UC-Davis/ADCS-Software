CC ?= gcc
CFLAGS ?= -MMD -MP -I.. -I.
CFLAGS += -g
CFLAGS += -Werror -Wall 
#CFLAGS += -Wextra 
CFLAGS += -Wno-unused-variable -Wno-unused-value -Wno-unused-but-set-variable

ifneq ($(V), 1)
Q = @
endif