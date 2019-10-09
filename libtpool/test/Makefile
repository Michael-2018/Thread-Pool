DIR_TESTC=./testc
DIR_TESTCPP=./testcpp

all: prepare
	make -C $(DIR_TESTC) ROOTDIR=$(ROOTDIR) DESTDIR=$(DESTDIR) || exit 1;\
	make -C $(DIR_TESTCPP) ROOTDIR=$(ROOTDIR) DESTDIR=$(DESTDIR) || exit 1;\

install:
	make -C $(DIR_TESTC) install ROOTDIR=$(ROOTDIR) DESTDIR=$(DESTDIR) || exit 1;\
	make -C $(DIR_TESTCPP) install ROOTDIR=$(ROOTDIR) DESTDIR=$(DESTDIR) || exit 1;\

clean:
	make -C $(DIR_TESTC) clean ROOTDIR=$(ROOTDIR) DESTDIR=$(DESTDIR) || exit 1;\
	make -C $(DIR_TESTCPP) clean ROOTDIR=$(ROOTDIR) DESTDIR=$(DESTDIR) || exit 1;\

prepare:
