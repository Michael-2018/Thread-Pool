#
# Bin
#
BIN_NAME   = tpool_testc
VERSION    = 1.0.0

#
# Target
#
TARGET = $(BIN_NAME)-$(VERSION)

#
# Message: Install ok
#
INSTALL_OK_MSG = "Install $(TARGET) OK!"

#
# path: source code
#
DIR_TPOOL_INC = $(ROOTDIR)/inc
DIR_TPOOL_SRC = $(ROOTDIR)/src

#
# path: test
#
DIR_TEST = $(ROOTDIR)/test/testc

#
# build custom flags
#
FLAGS += -I$(DIR_TPOOL_INC)
FLAGS += -I$(DIR_TPOOL_SRC)
FLAGS += -I$(DIR_TEST)

FLAGS += -fPIC -O2

#
# build compile and link flags
#
CFLAGS   += $(FLAGS)
CXXFLAGS += -std=c++11 $(FLAGS)
LDFLAGS  += -lpthread

SRC_FILES  = $(shell find $(DIR_TPOOL_SRC) -name "*.c" -o -name "*.cpp")
SRC_FILES += $(shell find $(DIR_TEST) -name "*.c" -o -name "*.cpp")

OBJS      += $(patsubst %.c, %.o, $(patsubst %.cpp, %.o, $(SRC_FILES)))

.PHONY: all clean install

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $^ $(LDFLAGS) -o $@

clean:
	-rm -f $(OBJS)
	-rm -f $(TARGET)

install:
	cp $(TARGET) $(DESTDIR)
	echo $(INSTALL_OK_MSG) 
#end
