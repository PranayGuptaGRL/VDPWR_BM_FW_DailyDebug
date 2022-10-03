## Copyright Cypress Semiconductor Corporation, 2014-2016,
## All Rights Reserved
## UNPUBLISHED, LICENSED SOFTWARE.
##
## CONFIDENTIAL AND PROPRIETARY INFORMATION 
## WHICH IS THE PROPERTY OF CYPRESS.
##
## Use of this file is governed 
## by the license agreement included in the file 
##
##	<install>/license/license.txt
##
## where <install> is the Cypress software
## installation root directory path.
## 

MODULE=libccgx_hpi
LIBEXT=a

ifeq ($(VERBOSE), yes)
    AT=
else
    AT=@
endif

CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

INCLUDE_PATHS=-I. -I./internal -I../system -I../scb -I../pd_common -I../app -I../app/alt_mode


ifeq ($(BOOT), yes)
    ifeq ($(FAMILY), ccg3)
        PREPROCESSOR_DEFINES=-DCCG3 -DDEBUG -DCCG_BOOT=1
        OUTDIR=ccg3_boot
    else
        PREPROCESSOR_DEFINES=-DCCG4 -DDEBUG -DCCG_BOOT=1
        OUTDIR=ccg4_boot
    endif
else
    PREPROCESSOR_DEFINES=-DDEBUG -DCCG_HPI_ENABLE=1 -DCCG_HPI_PD_ENABLE=1           \
                         -DAPP_PRIORITY_FEATURE_ENABLE=1                            \
                         -DDFP_ALT_MODE_SUPP=1 -DUFP_ALT_MODE_SUPP=1                \
                         -DCCG_HPI_BB_ENABLE=1
    ifeq ($(FAMILY), ccg3)
        PREPROCESSOR_DEFINES += -DCCG3 -DNO_OF_TYPEC_PORTS=1
        OUTDIR=ccg3
    else
        ifeq ($(DUALPORT), yes)
            PREPROCESSOR_DEFINES += -DCCG4 -DCCG_PD_DUALPORT_ENABLE -DNO_OF_TYPEC_PORTS=2
            OUTDIR=ccg4_dualport
        else
            PREPROCESSOR_DEFINES += -DCCG4 -DNO_OF_TYPEC_PORTS=1
            OUTDIR=ccg4_singleport
        endif
    endif
endif

LIBDIR=../../lib/$(OUTDIR)

CCFLAGS=-mcpu=cortex-m0 -mthumb -Wno-main -g -Wall -Werror \
	-ffunction-sections -Os -ffat-lto-objects -fno-strict-aliasing \
	$(INCLUDE_PATHS) $(PREPROCESSOR_DEFINES)

EXTSRC=../scb/i2c.c

SOURCE=i2c.c                    \
	hpi.c

OBJECT=$(SOURCE:%.c=$(OUTDIR)/%.o)

all: compile

copy:
	@echo "Copying external sources"
	@rm -f i2c.c
	cp $(EXTSRC) .

$(OUTDIR)/%.o : %.c
	@"mkdir" -p $(OUTDIR)
	@echo "Compiling $<"
	$(AT) $(CC) $(CCFLAGS) -c $< -o $@

clean:
	@echo === $@ ... ===
	@"date"
	@rm -rfv $(OUTDIR) $(LIBDIR)/$(MODULE).$(LIBEXT)
	@echo === ... $@ ===

compile: copy $(OUTDIR)/$(MODULE).$(LIBEXT)

$(OUTDIR)/$(MODULE).$(LIBEXT): $(OBJECT)
	@rm -rf $@
	@rm -rf $(LIBDIR)/$(MODULE).$(LIBEXT)
	@"mkdir" -p $(LIBDIR)
	@echo "Creating library $@"
	$(AT) $(AR) -cr $@ $+
	$(AT) cp -v $@ $(LIBDIR)

#[]#

