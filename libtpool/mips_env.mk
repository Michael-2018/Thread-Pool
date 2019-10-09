# 
# Toolchain path
#
DIR_TOOLCHAIN := 

#
# Prefix of tool
#
PREFIX = $(DIR_TOOLCHAIN)mips-linux-

export AR      := $(PREFIX)ar
export AS      := $(PREFIX)as
export CXX     := $(PREFIX)g++
export CC      := $(PREFIX)gcc
export NM      := $(PREFIX)nm
export OBJCOPY := $(PREFIX)objcopy
export OBJDUMP := $(PREFIX)objdump
export STRINGS := $(PREFIX)strings
export SSTRIP  := $(PREFIX)sstrip
export LSTRIP  := $(PREFIX)lstrip
export STRIP   := $(PREFIX)strip
