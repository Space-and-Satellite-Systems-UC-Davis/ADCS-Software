CC := gcc
CFLAGS := -I. -lm -Wswitch

# Source files
SOURCES := 


# Target executable name
TARGET := pointdish

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(SOURCES) $(CFLAGS) -o $@

run:
	./$(TARGET)

clean:
	rm -f $(TARGET)


.PHONY: all clean
