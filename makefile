include config.mk
include sources.mk

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

# ── Tests ─────────────────────────────────────────────────────────────────────

TEST_DIR   := tests
TEST_FLAGS := -g -Wall -Wextra -Wno-unused-variable -Wno-unused-value \
              -Wno-unused-but-set-variable -I.
VI_STUBS   := $(TEST_DIR)/stubs/vi_stubs.c

TEST_BINS := \
	$(TEST_DIR)/bin/test_vector \
	$(TEST_DIR)/bin/test_matrix \
	$(TEST_DIR)/bin/test_quaternion \
	$(TEST_DIR)/bin/test_calibration \

$(TEST_DIR)/bin:
	@mkdir -p $@

# adcs_math
$(TEST_DIR)/bin/test_vector: $(TEST_DIR)/adcs_math/test_vector.c adcs_math/vector.c | $(TEST_DIR)/bin
	@echo "CC $@"
	$(Q)$(CC) $(TEST_FLAGS) $^ -o $@ -lm

$(TEST_DIR)/bin/test_matrix: $(TEST_DIR)/adcs_math/test_matrix.c adcs_math/matrix.c adcs_math/vector.c | $(TEST_DIR)/bin
	@echo "CC $@"
	$(Q)$(CC) $(TEST_FLAGS) $^ -o $@ -lm

$(TEST_DIR)/bin/test_quaternion: $(TEST_DIR)/adcs_math/test_quaternion.c adcs_math/quaternion.c adcs_math/vector.c | $(TEST_DIR)/bin
	@echo "CC $@"
	$(Q)$(CC) $(TEST_FLAGS) $^ -o $@ -lm

$(TEST_DIR)/bin/test_calibration: $(TEST_DIR)/adcs_math/test_calibration.c adcs_math/calibration.c adcs_math/vector.c $(VI_STUBS) | $(TEST_DIR)/bin
	@echo "CC $@"
	$(Q)$(CC) $(TEST_FLAGS) $^ -o $@ -lm

test: $(TEST_BINS)
	@for t in $(TEST_BINS); do ./$$t || exit 1; done

clean:
	@echo "CLEAN"
	$(Q)rm -f $(TARGET) $(OBJECTS) $(DEPENDENCIES)
	$(Q)rm -rf $(TEST_DIR)/bin

.PHONY: all clean run test

