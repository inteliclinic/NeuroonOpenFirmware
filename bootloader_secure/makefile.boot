BOOT_PROJECT_NAME	:= neuroon-dfu-bootloader
PROJ_DIR := .
BOOT_PROJ_DIR 	      	:= bootloader_secure
BOOT_TARGET             := nrf51822_xxac_s130_boot
OUTPUT_DIRECTORY      	:= $(BOOT_PROJ_DIR)/_build

NEUROON_APP			:= $(PROJ_DIR)/_build/neuroonOpen.hex
#../../NeuroonOpenFirmware/_build/neuroonOpen.hex
NEUROON_APP_VER  		:= 0
NEUROON_BOOT_VER  		:= 0
NEUROON_BOOT_SETTINGS_VER  	:= 1 #has to be at least "1"

#SDK_ROOT := ../sdk

$(OUTPUT_DIRECTORY)/$(BOOT_TARGET).out: \
  LINKER_SCRIPT  := $(BOOT_PROJ_DIR)/secure_dfu_gcc_nrf51.ld

# Source files common to all targets
SRC_FILES = \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer_appsh.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/crc32/crc32.c \
  $(SDK_ROOT)/components/libraries/fstorage/fstorage.c \
  $(SDK_ROOT)/components/libraries/hci/hci_mem_pool.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/queue/nrf_queue.c \
  $(SDK_ROOT)/components/libraries/util/sdk_errors.c \
  $(SDK_ROOT)/components/libraries/sha256/sha256.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/rng/nrf_drv_rng.c \
  $(SDK_ROOT)/components/drivers_nrf/twi_master/nrf_drv_twi.c \
  $(SDK_ROOT)/components/drivers_nrf/hal/nrf_nvmc.c \
  $(SDK_ROOT)/components/libraries/bootloader/ble_dfu/nrf_ble_dfu.c \
  $(BOOT_PROJ_DIR)/dfu_req_handling.c \
  $(BOOT_PROJ_DIR)/main.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.S \
  $(SDK_ROOT)/components/toolchain/system_nrf51.c \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler_appsh.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader_app_start.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader_info.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_flash.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_mbr.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_settings.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_transport.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_utils.c \
  $(SDK_ROOT)/components/libraries/ecc/ecc.c \
  $(BOOT_PROJ_DIR)/dfu-cc.pb.c \
  $(BOOT_PROJ_DIR)/dfu_public_key.c \
  $(SDK_ROOT)/external/nano-pb/pb_common.c \
  $(SDK_ROOT)/external/nano-pb/pb_decode.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto.c \
  #$(BOOT_PROJ_DIR)/ic_pwr_management.c \
  #$(SDK_ROOT)/components/libraries/twi/app_twi.c \
  #$(BOOT_PROJ_DIR)/ic_driver_twi.c \

# Include folders common to all targets
INC_FOLDERS = \
  $(PROJ_DIR)/inc \
  $(SDK_ROOT)/components/drivers_nrf/rng \
  $(SDK_ROOT)/components/drivers_nrf/twi_master \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/libraries/sha256 \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(SDK_ROOT)/components/softdevice/s130/headers \
  $(BOOT_PROJ_DIR) \
  $(BOOT_PROJ_DIR)/config \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/bootloader/dfu \
  $(SDK_ROOT)/components/drivers_nrf/delay \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components/libraries/svc \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/libraries/bootloader \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/external/micro-ecc/micro-ecc \
  $(SDK_ROOT)/components/libraries/bootloader/ble_dfu \
  $(SDK_ROOT)/external/nano-pb \
  $(SDK_ROOT)/components/libraries/ecc \
  $(SDK_ROOT)/components/softdevice/s130/headers/nrf51 \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/components/libraries/twi \

# Libraries common to all targets
LIB_FILES = \
  $(SDK_ROOT)/external/micro-ecc/nrf51_armgcc/armgcc/micro_ecc_lib_nrf51.a \

# C flags common to all targets
CFLAGS = -DSWI_DISABLE0
#CFLAGS += -DBOARD_PCA10028
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DSVC_INTERFACE_CALL_AS_NORMAL_FUNCTION
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DNRF51822
CFLAGS += -DNRF_SD_BLE_API_VERSION=2
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror -Os -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums -flto
#CFLAGS += -fomit-frame-pointer

# C++ flags common to all targets
CXXFLAGS = \

# Assembler flags common to all targets
ASMFLAGS = -x assembler-with-cpp
ASMFLAGS += -DSWI_DISABLE0
#ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -DSVC_INTERFACE_CALL_AS_NORMAL_FUNCTION
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DNRF51822
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=2

# Linker flags
LDFLAGS = -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys


.PHONY: $(BOOT_TARGET) boot boot_gen_pub_key boot_gen_prv_key boot_generate_settings boot_merge_settings boot_flash boot_bin boot_clean
	# default all clean help

# Default target - first one defined
default: $(BOOT_PROJ_DIR)/dfu_public_key.c $(BOOT_TARGET)

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(BOOT_TARGET), $(call define_target, $(target)))

boot: $(BOOT_PROJ_DIR)/dfu_public_key.c $(OUTPUT_DIRECTORY)/$(BOOT_TARGET).hex

boot_clean: clean
	rm $(BOOT_PROJ_DIR)/dfu_public_key.c
	rm $(BOOT_PROJ_DIR)/private.pem

boot_bin: $(BOOT_PROJ_DIR)/dfu_public_key.c $(OUTPUT_DIRECTORY)/$(BOOT_TARGET).out
	@$(OBJCOPY) -O binary "$<" "$(OUTPUT_DIRECTORY)/$(BOOT_TARGET).bin"

boot_generate_settings: $(OUTPUT_DIRECTORY)/boot_settings.hex
$(OUTPUT_DIRECTORY)/boot_settings.hex:
	@test -s $(NEUROON_APP) || { echo "Build neuroonOpen.hex! Try <<make>>"; exit 1; }
	@echo Generating bootloader settings.
	nrfutil settings generate --family NRF51 --application $(NEUROON_APP) --application-version $(NEUROON_APP_VER) --bootloader-version $(NEUROON_BOOT_VER) --bl-settings-version $(NEUROON_BOOT_SETTINGS_VER) $(@D)/boot_settings.hex

boot_merge_settings: $(OUTPUT_DIRECTORY)/$(BOOT_TARGET)_s.hex
$(OUTPUT_DIRECTORY)/$(BOOT_TARGET)_s.hex: $(BOOT_PROJ_DIR)/dfu_public_key.c $(addprefix $(OUTPUT_DIRECTORY)/, $(BOOT_TARGET).hex boot_settings.hex)
	@echo Merging bootloader with bootloader settings.
	mergehex -m $(addprefix $(OUTPUT_DIRECTORY)/, $(BOOT_TARGET).hex boot_settings.hex) -o $(OUTPUT_DIRECTORY)/$(BOOT_TARGET)_s.hex

# Flash the program
boot_flash: $(OUTPUT_DIRECTORY)/$(BOOT_TARGET)_s.hex
	@echo Flashing dfu-bootloader with settings: $<
	nrfjprog --program $< -f nrf51 --sectorerase
	nrfjprog --reset -f nrf51

# Keys generating
boot_gen_prv_key: $(BOOT_PROJ_DIR)/private.pem
$(BOOT_PROJ_DIR)/private.pem:
	@echo Checking private key.
	@test -s $(BOOT_PROJ_DIR)/private.pem || { nrfutil keys generate $(BOOT_PROJ_DIR)/private.pem; }

boot_gen_pub_key: $(BOOT_PROJ_DIR)/dfu_public_key.c
$(BOOT_PROJ_DIR)/dfu_public_key.c: $(BOOT_PROJ_DIR)/private.pem
	@echo Checking public key.
	@test -s $(BOOT_PROJ_DIR)/dfu_public_key.c || { nrfutil keys display --key pk --format code $(BOOT_PROJ_DIR)/private.pem --out_file $(BOOT_PROJ_DIR)/dfu_public_key.c; }

