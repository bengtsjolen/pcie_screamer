include ../include/generated/variables.mak

LIBBASE_DIRECTORY ?= $(SOC_DIRECTORY)/software/libbase

include $(SOC_DIRECTORY)/software/common.mak

OBJECTS = main.o crt0.o
LSCRIPT = linker.ld

all: bios.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

bios.elf: $(BIOS_DIRECTORY)/$(LSCRIPT) $(OBJECTS)
	$(CC) $(LDFLAGS) -T $(BIOS_DIRECTORY)/$(LSCRIPT) -N -o $@ $(OBJECTS)

VPATH = $(BIOS_DIRECTORY):$(CPU_DIRECTORY)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

$(BUILDINC_DIRECTORY)/../libc/picolibc.h: $(SOC_DIRECTORY)/software/libc/picolibc-minimal.h
	@mkdir -p $(dir $@)
	cp $< $@
	printf "#define FORMAT_DEFAULT_%s\n" "$(shell echo $(PICOLIBC_FORMAT) | tr a-z A-Z)" >> $@
	printf "#define __IO_DEFAULT '%s'\n" "$(shell printf '%s' "$(PICOLIBC_FORMAT)" | cut -c1)" >> $@

main.o: $(BUILDINC_DIRECTORY)/../libc/picolibc.h

clean:
	$(RM) $(OBJECTS) bios.elf bios.bin

.PHONY: all clean

-include $(OBJECTS:.o=.d)
