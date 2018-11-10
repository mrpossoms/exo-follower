#----------------------------------------
#   __   __        _      _    _
#   \ \ / /_ _ _ _(_)__ _| |__| |___ ___
#    \ V / _` | '_| / _` | '_ \ / -_|_-<
#     \_/\__,_|_| |_\__,_|_.__/_\___/__/
#
include $(EXO_ROOT)/vars.mk

MOD_NAME=$(shell basename $(shell pwd))
MOD_SRC=$(shell ls *.cpp)
OS_TARGET=$(exo get os_target)
TARGET_OBJS=$(shell ls $(EXO_ROOT)/obj/$(TARGET)/$(OS_TARGET)_*)
MOD_OBJS=$(addprefix obj/$(TARGET)/,$(MOD_SRC:.cpp=.o))
INC+=-I../../
FLAGS+=$(shell exo magicdef)

#----------------------------------------
#    ___      _
#   | _ \_  _| |___ ___
#   |   / || | / -_|_-<
#   |_|_\\_,_|_\___/__/
#
build:
	./setup.sh

.PHONY: all
all: build exo
	make -C build
	@echo "$(COLOR_GREEN) Built gazebo plugin for $(TARGET) $(COLOR_OFF)"

.PHONY: clean
clean:
	rm -rf build obj bin
