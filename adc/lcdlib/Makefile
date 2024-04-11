#
# Copyright (c) 2017 Intel Corporation
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0
#

PREFIX ?= .
OBJ_DIR ?= $(PREFIX)/obj
LIB_DIR ?= $(PREFIX)/lib

ifeq ($(findstring Windows, $(OS)),Windows)
	DEL := rmdir /S /Q
	MKDIR := mkdir
else
	DEL := rm -rf
	MKDIR := mkdir -p
endif

all:
	-$(MKDIR) "$(OBJ_DIR)"
	-$(MKDIR) "$(LIB_DIR)"
	$(CC) -c $(CFLAGS) -MD -Iinclude src/font8.c -o $(OBJ_DIR)/mylib1.o
	$(CC) -c $(CFLAGS) -MD -Iinclude src/font12.c -o $(OBJ_DIR)/mylib2.o
	$(CC) -c $(CFLAGS) -MD -Iinclude src/font16.c -o $(OBJ_DIR)/mylib3.o
	$(CC) -c $(CFLAGS) -MD -Iinclude src/font20.c -o $(OBJ_DIR)/mylib4.o
	$(CC) -c $(CFLAGS) -MD -Iinclude src/font24.c -o $(OBJ_DIR)/mylib5.o
	$(CC) -c $(CFLAGS) -MD -Iinclude src/ili9341.c -o $(OBJ_DIR)/mylib6.o
	$(CC) -c $(CFLAGS) -MD -Iinclude src/stm32f429i_discovery.c -o $(OBJ_DIR)/mylib7.o
	$(CC) -c $(CFLAGS) -MD -Iinclude src/stm32f429i_discovery_lcd.c -o $(OBJ_DIR)/mylib8.o
	$(CC) -c $(CFLAGS) -MD -Iinclude src/stm32f429i_discovery_sdram.c -o $(OBJ_DIR)/mylib9.o
	$(AR) -rcs $(LIB_DIR)/libmylib.a $(OBJ_DIR)/mylib1.o $(OBJ_DIR)/mylib2.o $(OBJ_DIR)/mylib3.o $(OBJ_DIR)/mylib4.o $(OBJ_DIR)/mylib5.o $(OBJ_DIR)/mylib6.o $(OBJ_DIR)/mylib7.o $(OBJ_DIR)/mylib8.o $(OBJ_DIR)/mylib9.o

clean:
	$(DEL) "$(OBJ_DIR)" "$(LIB_DIR)"
