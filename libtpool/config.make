#
# MIPS, X86
#
ARCH = X86 

ifeq ($(ARCH), MIPS)
    include mips_env.mk
else ifeq ($(ARCH), X86)
    include x86_env.mk
else
    include x86_env.mk
endif
