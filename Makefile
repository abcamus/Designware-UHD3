INCLUDE := -Iinclude/ -I./
SRC_C := usb.c \
	host/xhci-dwc3.c \
	host/xhci.c \
	host/xhci-mem.c \
	host/xhci-dev.c \
	host/xhci-ring.c \
	lib/ctype.c \
	usb_hub.c \
	usb_storage.c
TARGET := dwc3
BUILD := build
PREFIX := aarch64-linux-gnu-
CC := $(PREFIX)gcc
CFLAGS += -O0

OBJS := $(SRC_C:%.c=%.o)
#LINK_OBJS := $(addprefix $(BUILD)/, $(patsubst %.c, %.o, $(notdir $(SRC_C))))
#$(warning $(notdir $(OBJS)))

all: $(TARGET)

$(TARGET): $(OBJS)
	@$(CC) $(CFLAGS) -o $@ $(INCLUDE) $(addprefix $(BUILD)/, $(notdir $(OBJS)))

%.o:%.c |$(BUILD)
	@$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $@
	@mv $@ $(BUILD)

$(BUILD):
	mkdir $(BUILD)

.PHONY: clean, distclean, pack
clean:
	@rm -rf $(BUILD)/*.o $(TARGET) log

distclean:
	@rm -rf $(BUILD) $(TARGET) $(TARGET).tar.bz2 log tags

pack:
	@tar acvf $(TARGET).tar.bz2 *
