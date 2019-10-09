include ./config.make

PWD=$(shell pwd)

ROOTDIR=$(PWD)
DESTDIR=$(PWD)/bin
TEST_DIR=$(PWD)/test

all: prepare
	make -C $(TEST_DIR) ROOTDIR=$(ROOTDIR) DESTDIR=$(DESTDIR) || exit 1;\

install:
	make -C $(TEST_DIR) install ROOTDIR=$(ROOTDIR) DESTDIR=$(DESTDIR) || exit 1;\

clean:
	make -C $(TEST_DIR) clean ROOTDIR=$(ROOTDIR) DESTDIR=$(DESTDIR) || exit 1;\
	rm -rf $(DESTDIR)

prepare:
	mkdir -p $(DESTDIR)
