# Kernel Modules Makefile

# Kernel source and build directories
KERNEL_SRC := $(PWD)/workspace/kernel  
MODULES_DIR := $(KERNEL_SRC)/drivers/extra
TARGET_ARCH := arm64
KERNEL_MODULES := \
	bt_drv_connac1x \
	wmt_drv \
	connfem \
	conninfra \
	fmradio_drv_mt6631 \
	gps_drv \
	wmt_chrdev_wifi \
	wlan_drv_gen4m \
	fpsgo \
	met \
	udc_lib

# Kernel build configuration
KERNEL_DEFCONFIG := $(KERNEL_SRC)/TB125FU_debug_defconfig
CC := clang 
CROSS_COMPILE := aarch64-linux-android-

.PHONY: all clean modules $(KERNEL_MODULES)

# Default target
all: modules

# Build all kernel modules
modules: $(KERNEL_MODULES)

# Compile the individual modules
$(KERNEL_MODULES):
	@echo "Building module: $@"
	@make -C $(KERNEL_SRC) O=$(OUT_DIR) ARCH=$(TARGET_ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(MODULES_DIR)/$@ modules 2>&1 | tee -a build.log

# Clean the build directory
clean:
	@echo "Cleaning kernel modules..."
	@make -C $(KERNEL_SRC) O=$(OUT_DIR) ARCH=$(TARGET_ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(MODULES_DIR) clean

