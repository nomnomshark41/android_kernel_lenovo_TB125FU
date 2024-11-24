# Kernel source and build directories
KERNEL_SRC := $(PWD)/workspace/kernel
MODULES_DIR := $(KERNEL_SRC)/drivers/extra
TARGET_ARCH := arm64
KERNEL_MODULES := $(shell find $(MODULES_DIR) -type f \( -name "Makefile" -o -name "Android.mk" \) -exec dirname {} \;)

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
	@if [ -f $(MODULES_DIR)/$@/Android.mk ]; then \
		echo "Building using Android.mk"; \
		$(MAKE) -C $(MODULES_DIR)/$@ -f $(MODULES_DIR)/$@/Android.mk; \
	elif [ -f $(MODULES_DIR)/$@/Makefile ]; then \
		echo "Building using Makefile"; \
		make -C $(KERNEL_SRC) O=$(OUT_DIR) ARCH=$(TARGET_ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(MODULES_DIR)/$@ modules; \
	else \
		echo "No build system found for module: $@"; \
	fi

# Clean the build directory
clean:
	@echo "Cleaning kernel modules..."
	@make -C $(KERNEL_SRC) O=$(OUT_DIR) ARCH=$(TARGET_ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(MODULES_DIR) clean
