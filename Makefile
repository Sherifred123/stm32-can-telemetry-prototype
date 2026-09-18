# ==============================================================================
# Makefile - STM32 CAN 2.0B Communication & Telemetry Prototype
# Author: S. Sherifred Singh (Senior Embedded Firmware Engineer)
# ==============================================================================

CC      := gcc
CFLAGS  := -std=c99 -Wall -Wextra -pedantic -O2 -Iinclude
SRC     := src/can_driver.c src/can_telemetry.c src/main.c
TARGET  := can_demo

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SRC)
	@echo "[CC] Compiling $(TARGET)..."
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET)
	@echo "[OK] Build complete: ./$(TARGET)"

run: all
	@echo "[EXEC] Running CAN Telemetry Prototype..."
	./$(TARGET)

clean:
	@echo "[CLEAN] Removing artifacts..."
	rm -f $(TARGET) $(TARGET).exe
