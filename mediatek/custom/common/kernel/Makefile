include $(MTK_PATH_BUILD)/common.mk
$(call all-modules-src-or-makefile,$(obj),CUSTOM_KERNEL_)
obj-n := dummy.o

ifeq ($(PROJECT),yaris_xxl)
EXTRA_CFLAGS += -DYARISXXL
endif
