# nuttx build
ifneq ($(APPDIR),)
MENUDESC = "Antenna Tracker"

include $(APPDIR)/Directory.mk

# desktop build
else

TARGET = tracker

.PHONY: $(TARGET)

all: $(TARGET)

$(TARGET):
	$(MAKE) -C $(abspath $@)

clean:
	$(MAKE) -C $(TARGET) clean

endif
