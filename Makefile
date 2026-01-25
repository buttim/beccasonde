PROJECT := $(notdir $(CURDIR))

$(PROJECT)_SOURCE := $(wildcard *.c) $(wildcard *.cpp)  \
    $(TREMO_SDK_PATH)/platform/system/system_cm4.c  \
    $(TREMO_SDK_PATH)/platform/system/startup_cm4.S \
    $(TREMO_SDK_PATH)/platform/system/printf-stdarg.c  \
    $(wildcard $(TREMO_SDK_PATH)/lora/radio/sx126x/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/lora/driver/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/lora/system/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/drivers/peripheral/src/*.c)

$(PROJECT)_INC_PATH := . ~/Reed-Solomon/include  \
    $(TREMO_SDK_PATH)/platform/CMSIS \
    $(TREMO_SDK_PATH)/platform/common \
    $(TREMO_SDK_PATH)/platform/system \
    $(TREMO_SDK_PATH)/lora/driver  \
    $(TREMO_SDK_PATH)/lora/system  \
    $(TREMO_SDK_PATH)/lora/radio  \
    $(TREMO_SDK_PATH)/lora/radio/sx126x  \
    $(TREMO_SDK_PATH)/drivers/peripheral/inc

$(PROJECT)_CFLAGS  := -Wall -Os -ffunction-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -fsingle-precision-constant -std=c2x -fno-builtin-printf -fno-builtin-sprintf -fno-builtin-snprintf -fno-math-errno 
$(PROJECT)_CXXFLAGS  := -Wall -Os -ffunction-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -fsingle-precision-constant -fno-builtin-printf -fno-builtin-sprintf -fno-builtin-snprintf -fno-math-errno 
$(PROJECT)_DEFINES := -DCONFIG_DEBUG_UART=UART0

$(PROJECT)_LDFLAGS := -Wl,--gc-sections -Wl,--wrap=printf -Wl,--wrap=sprintf -Wl,--wrap=snprintf  --specs=rdimon.specs 
$(PROJECT)_LIBS := 

$(PROJECT)_LINK_LD := cfg/gcc.ld

# please change the settings to download the app
#SERIAL_PORT        := 
#SERIAL_BAUDRATE    :=
#$(PROJECT)_ADDRESS := 

##################################################################################################
include $(TREMO_SDK_PATH)/build/make/common.mk



