include config.mk
include sources.mk

# Optional host-specific overrides (gitignored). Used for things like pointing
# Homebrew GCC at the macOS SDK; see local.mk.example.
-include local.mk

# Target executable name
TARGET := libADCS.a

all: $(TARGET)

DEPENDENCIES := $(patsubst %.o,%.d,$(OBJECTS))
-include $(DEPENDENCIES)

$(TARGET): $(OBJECTS)
	@echo "AR $@"
	$(Q)ar rcs $@ $^

%.o: %.c
	@echo "CC $@"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@ 

run:
	./$(TARGET)

version:
	$(Q)$(CC) --version

clean:
	@echo "CLEAN"
	$(Q)rm -f $(TARGET) $(OBJECTS) $(DEPENDENCIES)

.PHONY: all clean run version

