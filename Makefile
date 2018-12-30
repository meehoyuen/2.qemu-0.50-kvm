include config.mak

TARGET_PATH=$(SRC_PATH)/target-$(TARGET_ARCH)
VPATH=$(SRC_PATH):$(TARGET_PATH)
CFLAGS=-O0 -g
LDFLAGS=-g
LIBS=
DEFINES=-I. -I$(TARGET_PATH) -I$(SRC_PATH)
HELPER_CFLAGS=$(CFLAGS)
DYNGEN=../dyngen
# user emulator name
QEMU_USER=qemu-$(TARGET_ARCH)
# system emulator name
QEMU_SYSTEM=qemu

ifdef CONFIG_USER_ONLY
PROGS=$(QEMU_USER)
else
ifeq ($(ARCH),i386)
ifeq ($(TARGET_ARCH), i386)
PROGS+=$(QEMU_SYSTEM)
endif
endif
endif

ifdef CONFIG_STATIC
LDFLAGS+=-static
endif

ifeq ($(ARCH),i386)
CFLAGS+=-fomit-frame-pointer
OP_CFLAGS=$(CFLAGS) -mpreferred-stack-boundary=4
ifeq ($(HAVE_GCC3_OPTIONS),yes)
OP_CFLAGS+= -falign-functions=0
else
OP_CFLAGS+= -malign-functions=0
endif
ifdef TARGET_GPROF
LDFLAGS+=
else
# WARNING: this LDFLAGS is _very_ tricky : qemu is an ELF shared object
# that the kernel ELF loader considers as an executable. I think this
# is the simplest way to make it self virtualizable!
LDFLAGS+=-Wl,-shared
endif
endif

ifeq ($(HAVE_GCC3_OPTIONS),yes)
# very important to generate a return at the end of every operation
OP_CFLAGS+=-fno-reorder-blocks -fno-optimize-sibling-calls
endif

#########################################################

DEFINES+=-D_GNU_SOURCE
LIBS+=-lm

SRCS:= $(OBJS:.o=.c)

all: $(PROGS)

$(QEMU_USER): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^  $(LIBS)

# must use static linking to avoid leaving stuff in virtual address space
VL_OBJS=vl.o block.o vga.o
ifdef CONFIG_SDL
VL_OBJS+=sdl.o
SDL_LIBS+=-L/usr/X11R6/lib -lX11 -lXext -lXv -ldl -lm
endif

$(QEMU_SYSTEM): $(VL_OBJS)
	$(CC) -static -o $@ $^ $(LIBS) $(SDL_LIBS) -lpthread

sdl.o: sdl.c
	$(CC) $(CFLAGS) $(DEFINES) $(SDL_CFLAGS) -c -o $@ $<

depend: $(SRCS)
	$(CC) -MM $(CFLAGS) $(DEFINES) $^ 1>.depend

%.o: %.c
	$(CC) $(CFLAGS) $(DEFINES) -c -o $@ $<

clean:
	rm -f *.o  *.a *~ $(PROGS)

ifneq ($(wildcard .depend),)
include .depend
endif
