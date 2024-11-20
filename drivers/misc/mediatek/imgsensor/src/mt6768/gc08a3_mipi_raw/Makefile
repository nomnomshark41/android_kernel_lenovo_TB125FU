ARCH_MTK_P98980 := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

ifeq ($(ARCH_MTK_P98980),P98980AA1)
obj-y += ../../common/v1/gc08a3_mipi_raw/gc08a3mipi_Sensor.o
else

ifeq ($(ARCH_MTK_P98980),P98980EA1)
obj-y += ../../common/v1/gc08a3_mipi_raw/gc08a3mipi_Sensor.o
else
obj-y += gc08a3mipi_Sensor.o
endif

endif
