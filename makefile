# VEXcode makefile 2019_03_26_01

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# location of the project source cpp and c files
SRC_C  = $(wildcard src/*.cpp) 
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard src/*/*.cpp) 
SRC_C += $(wildcard src/*/*.c)

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(filter-out src/test/%, $(SRC_C)))) )

OBJ_TEST = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(filter-out src/main.cpp, $(SRC_C)))))


# location of include files that c and cpp files depend on
SRC_H  = $(wildcard include/*.h)

# additional dependancies
SRC_A  = makefile

# project header file locations
INC_F  = include

# build targets
all: $(BUILD)/$(PROJECT).bin

test: 
	$(MAKE) test_app \
		DEFINES="-DTEST" \
		CXX="g++" \
		CXX_FLAGS="-Os -Wall -DTEST" \
		INC="-I./include" \
		LIBS="-lc -lm" \
		BUILD="build/test" \
		PLATFORM="TEST"

test_app: $(BUILD)/$(PROJECT)-test


upload: $(BUILD)/$(PROJECT).bin
	$(VEXCOM) --slot $(SLOT) --write $(BUILD)/$(PROJECT).bin  
	

# include build rules
include vex/mkrules.mk
