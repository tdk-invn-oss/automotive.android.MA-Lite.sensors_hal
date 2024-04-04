# HAL source files location
HAL_SRC_DIR := ../..

include $(HAL_SRC_DIR)/invensense.mk

# Use LLVM libc++ with Clang, GNU libstdc++ by default
ifeq ($(CXX), clang++)
CXX_STL = -lc++
CXXFLAGS += -stdlib=libc++
else
CXX_STL = -lstdc++
endif

# Common flags
CPPFLAGS += -DLOG_TAG=\"InvnSensors\"
CPPFLAGS += -Iandroid_linux -D_HW_DONT_INCLUDE_CORE_
CPPFLAGS += $(foreach inc, $(INV_INCLUDES), -I$(HAL_SRC_DIR)/$(inc))
COMMON_FLAGS += -O2
COMMON_FLAGS += -Wall -Wextra -Werror
COMMON_FLAGS += -fPIC -pthread
CFLAGS += -std=gnu11 $(COMMON_FLAGS)
CXXFLAGS += -std=gnu++11 $(COMMON_FLAGS)

# InvenSense Sensors HAL
INVNSENSORS := libinvnsensors.so
CPPFLAGS += $(INV_CFLAGS)
INVNSENSORS_SRC_CPP_FILES += $(foreach src_cpp, $(filter %.cpp,$(INV_SRCS)), $(HAL_SRC_DIR)/$(src_cpp))
INVNSENSORS_SRC_C_FILES += $(foreach src_c, $(filter %.c,$(INV_SRCS)), $(HAL_SRC_DIR)/$(src_c))
INVNSENSORS_LDFLAGS += $(LDFLAGS) -L./ -shared -lpthread $(CXX_STL)
INVNSENSORS_OBJ_FILES := $(INVNSENSORS_SRC_C_FILES:.c=.o) $(INVNSENSORS_SRC_CPP_FILES:.cpp=.o)

# test application
TESTAPP := test-sensors-hal
TESTAPP_SRC_CPP_FILES := test-sensors-hal.cpp
TESTAPP_LDFLAGS = $(LDFLAGS) -ldl -lpthread $(CXX_STL)
TESTAPP_OBJ_FILES := $(TESTAPP_SRC_C_FILES:.c=.o) $(TESTAPP_SRC_CPP_FILES:.cpp=.o)

.PHONY: all clean

all: $(INVNSENSORS) $(TESTAPP)

clean:
	-rm -f $(INVNSENSORS_OBJ_FILES) $(INVNSENSORS)
	-rm -f $(TESTAPP_OBJ_FILES) $(TESTAPP)

$(INVNSENSORS): $(MLLITE) $(INVNSENSORS_OBJ_FILES)
	$(CXX) $(CXXFLAGS) $^ $(INVNSENSORS_LDFLAGS) $(INVNSENSORS_LIBS) -o $@

$(TESTAPP): $(TESTAPP_OBJ_FILES)
	$(CXX) $(CXXFLAGS) $^ $(TESTAPP_LDFLAGS) -o $@
