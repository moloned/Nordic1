######################################
# For nRF51822 on Kepler Eclipse using the ARM GNU toolchain
# Supports both the assembler and C startup code stubs
# Liam Goudge April 2014

# Project Name
PROJECT = nRF51822

################### File Locations #####################
INCLUDES = -I./
ARMGNUMini = /Users/Liam/Documents/gcc-arm-none-eabi-4_7-2013q3/bin/
ARMGNULaptop = /Users/liamg/Documents/gcc-arm-none-eabi-4_8-2013q4/bin/
LINKER_SCRIPT = Settings/nRF51822.ld

# Compiler/Assembler/Linker Paths
PATH = 	$(ARMGNUMini)arm-none-eabi-

CC = 	$(PATH)gcc
AS = 	$(PATH)as
LD = 	$(PATH)ld
NM = 	$(PATH)nm
CP = 	$(PATH)objcopy
OD = 	$(PATH)objdump

OBJDIR = Objects

################### Compiler and Linker Settings #####################
OPT = -Os -g

# Compiler Options
CFLAGS = -fno-common -mcpu=cortex-m0 -mthumb
CFLAGS += $(OPT) $(DEBUG) $(INCLUDES)
CFLAGS += -Wall -Wextra
CFLAGS += -Wcast-align -Wcast-qual -Wimplicit -Wpointer-arith -Wswitch -Wredundant-decls -Wreturn-type -Wshadow -Wunused -c

# Linker options
LDFLAGS = -mcpu=cortex-m0 -mthumb $(OPT) -nostartfiles -Wl,-Map=$(PROJECT).map -T$(LINKER_SCRIPT) -nostdlib
LDFLAGS2 = -nostartfiles -T$(LINKER_SCRIPT) -nostdlib

# Assembler options
ASFLAGS = -mcpu=cortex-m0 -mthumb

CPFLAGS = -Obinary
ODFLAGS = -h --syms -S

REMOVE = rm -f

################### Targets #####################

all:
	@ echo " "
	@ echo "Compiling:"
	mkdir -p $(OBJDIR)
	$(AS) $(ASFLAGS) Settings/startup.s -o $(OBJDIR)/startup.o 	# Switch this in (and the C out) for assembler startup
	#$(CC) $(CFLAGS) Settings/startup.c -o $(OBJDIR)/startup.o		# Switch this in (and the assembler out) for C startup
	$(CC) $(CFLAGS) Settings/Initialize.c -o $(OBJDIR)/Initialize.o
	$(CC) $(CFLAGS) Sources/main.c -o Objects/main.o
	
	@ echo " "	
	@ echo "Linking:"
	$(CC) $(LDFLAGS)  $(OBJDIR)/startup.o $(OBJDIR)/Initialize.o $(OBJDIR)/main.o -o $(PROJECT).elf
		
	@ echo " "	
	@ echo "List file generation:"
	$(OD) $(ODFLAGS) $(PROJECT).elf > $(PROJECT).lst
	
	@ echo " "
	@ echo "Symbol table:"
	$(NM) $(PROJECT).elf
	
	@ echo " "
	@ echo "Cleaning up:"
	$(REMOVE) -r $(OBJDIR)

	
