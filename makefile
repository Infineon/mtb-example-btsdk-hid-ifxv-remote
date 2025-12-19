#
# (c) 2016-2025, Infineon Technologies AG, or an affiliate of Infineon
# Technologies AG. All rights reserved.
# This software, associated documentation and materials ("Software") is
# owned by Infineon Technologies AG or one of its affiliates ("Infineon")
# and is protected by and subject to worldwide patent protection, worldwide
# copyright laws, and international treaty provisions. Therefore, you may use
# this Software only as provided in the license agreement accompanying the
# software package from which you obtained this Software. If no license
# agreement applies, then any use, reproduction, modification, translation, or
# compilation of this Software is prohibited without the express written
# permission of Infineon.
# 
# Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
# IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
# THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
# SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
# Infineon reserves the right to make changes to the Software without notice.
# You are responsible for properly designing, programming, and testing the
# functionality and safety of your intended application of the Software, as
# well as complying with any legal requirements related to its use. Infineon
# does not guarantee that the Software will be free from intrusion, data theft
# or loss, or other breaches ("Security Breaches"), and Infineon shall have
# no liability arising out of any Security Breaches. Unless otherwise
# explicitly approved by Infineon, the Software may not be used in any
# application where a failure of the Product or any consequences of the use
# thereof can reasonably be expected to result in personal injury.
#

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif

$(info !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)
$(info !!!!!!!!                 WARNING                     !!!!!!!!!!!!!!!!)
$(info This sample application demonstrates a custom GATT service with)
$(info 16-bit service UUID 0x0000.  This is not a valid service ID and must)
$(info not be used in a product. Valid 16-bit service UUID must be purchased)
$(info from the Bluetooth SIG and code must be updated with the valid UUID.)
$(info See application README.md for more details.)
$(info !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)

#
# Basic Configuration
#
APPNAME=BLE_IFXV_Remote
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=

# default target
TARGET=CYW920835REF-RCU-01

SUPPORTED_TARGETS = \
    CYW920835M2EVB-01 \
    CYW920835REF-RCU-01 \
    CYBLE-343072-EVAL-M2B \
    CYBLE-333074-EVAL-M2B

ifeq ($(filter $(TARGET),$(SUPPORTED_TARGETS)),)
 $(error TARGET $(TARGET) is not supported for this application. Edit SUPPORTED_TARGETS in the code example makefile to add new BSPs)
endif

# When CUSTOM is defined as 1, the key matrix will be custom 3x5
# When CUSTOM is defined as 2, the key matrix will be custom 3x6
# otherwise, the bsp specified in TARGET will be used.
CUSTOM?=0
ifeq ($(CUSTOM),1)
 CY_APP_DEFINES += -DCUSTOM_KEY_MATRIX=1 -DSUPPORT_BUTTON -DSUPPORT_KEYSCAN
 COMPONENTS += bsp_custom_design_3x5
 $(info Building Custom BSP with key matrix 3x5)
else
 ifeq ($(CUSTOM),2)
  CY_APP_DEFINES += -DCUSTOM_KEY_MATRIX=2 -DSUPPORT_BUTTON -DSUPPORT_KEYSCAN
  COMPONENTS += bsp_custom_design_3x6
  $(info Building custom bsp with key matrix 3x6)
 else
  COMPONENTS += bsp_design_modus
  ifeq ($(filter $(TARGET), CYW920835REF-RCU-01),)
   CY_APP_DEFINES += -DWICED_EVAL -DSUPPORT_BUTTON
   $(info Building $(TARGET) bsp)
  else
   $(info Building $(TARGET) bsp with key matrix 3x7)
   CY_APP_DEFINES += -DSUPPORT_KEYSCAN
  endif
 endif
endif

#
# Advanced Configuration
#
SOURCES=
INCLUDES=
DEFINES=
VFP_SELECT=
CFLAGS=
CXXFLAGS=
ASFLAGS=
LDFLAGS=
LDLIBS=
LINKER_SCRIPT=
PREBUILD=
POSTBUILD=
FEATURES=

#
# Define basic library COMPONENTS
#
COMPONENTS += hidd_lib gatt_utils_lib

#######################################################################################
# App compile flag defaults
#
# All flags can be defined in command line. When it is not defined, the following default value is used.
# Set to 1 to enable, 0 to disable.
#

####################################################
XIP?=xip
BT_DEVICE_ADDRESS?=default
UART?=AUTO

####################################################
# To use ClientControl to control the device via HCI_UART port, TESTING_USING_HCI flag must be turned on
#
# BtSpy logs should work by enabling TESTING_USING_HCI. However,
# by enabling ENABLE_BT_SPY will route debug message from PUART to BtSpy (WICED_UART).
# TESTING_USING_HCI must be enabled for ENABLE_BT_SPY option to take effect.
#
TESTING_USING_HCI?=1
ENABLE_BT_SPY?=0

####################################################
# SLEEP_ALLOWED
#  SLEEP_ALLOWED=0  Disable sleep function
#  SLEEP_ALLOWED=1  Allow sleep without shutdown
#  SLEEP_ALLOWED=2  Allow sleep with shutdown
#
SLEEP_ALLOWED?=2

####################################################
# LED
#  LED=0  Disable LED functions (Good for power consumption measurement)
#  LED=1  Use LED for status indication
#
ifeq ($(CUSTOM),0)
 LED?=1
else
 LED?=0
endif

####################################################
# Use OTA_FW_UPGRADE=1 to enable Over-the-air firmware upgrade functionality
# Use OTA_SEC_FW_UPGRADE=1 in the make target to use secure OTA procedure.
# OTA_SEC_FW_UPGRADE_DEFAULT takes effect only when OTA_FW_UPGRADE_DEFAULT=1.
# When secure OTA is enabled, The ecdsa256_public_key content should be updated
# with the generated key in file ecdsa256_pub.c.
#
OTA_FW_UPGRADE?=1
OTA_SEC_FW_UPGRADE?=0

####################################################
# AUDIO defines
#
# AUDIO=XXXX
#    - Use this option to enable audio. Where XXXX is:
#
#      - IFXV       use Infineon Voice over GATT profile (VoGP)
#
# CODEC=XXXX
#    - Use this option to enable audio codec. The AUDIO option must be enabled for this option to take effect. Where XXXX is:
#
#      - OPUS       use OPUS CELT encoder
#      - ADPCM      use ADPCM encoder
#
# PDM=x
#    - Use this option to enable/disable digital microphone (PDM=1/0). The AUDIO option must be enabled for this option to take effect.
#    - This option is invalid for CYW920835REF-RCU-01 BSP as it does not have PDM hardware. In CYW920835M2EVB-01 platform,
#      since the GPIO pins are shared with LEDs, make sure SW4 is switched to DMIC and the LED is disabled for the complier option.
#
AUDIO?=IFXV
CODEC?=OPUS
PDM?=0

ifeq ($(AUDIO),ATV)
 # CODEC=ADPCM
 $(error Google Voice is not supported)
endif

####################################################
# Link related flags
#
# SKIP_PARAM_UPDATE
#   Use SKIP_PARAM_UPDATE=1 to not request connection parameter update immediately when
#   received LE conn param update complete event with non-preferred values.
#   Because audio requires shorter connection interval to steam data, skipping link parameter
#   update can cause audio not to work propertly until it is updated.
#
# AUTO_RECONNECT
#   Use AUTO_RECONNECT=1 to automatically reconnect when connection drops
#
# START_ADV_ON_POWERUP
#   Use START_ADV_ON_POWERUP=1 to start advertising on power up. If paired it reconnect when power up.
#
# ENABLE_CONNECTED_ADV
#   Use ENABLE_CONNECTED_ADV=1 to enable advertising while connected
#
# ENDLESS_ADV
#   Use ENDLESS_ADV=1 to enable endless advertising. Otherwise, the advertisement expires in given period.
#
# LE_LOCAL_PRIVACY
#   Use LE_LOCAL_PRIVACY=1 to advertise with Resolvable Private Address (RPA)
#
SKIP_PARAM_UPDATE?=0
AUTO_RECONNECT?=1
START_ADV_ON_POWERUP?=1
ENABLE_CONNECTED_ADV?=1
ENDLESS_ADV?=1
LE_LOCAL_PRIVACY=0

####################################################
# Use ENABLE_IR=1 to enable IR support.
#
# Do not enable for CYW920835REF-RCU-01 as P38 is used for battery monitoring in the hardware design.
# It can cause battery monitor malfunction and wrongfully to shutdown the device.
# To enable it, please change the code to use other GPIO.
#
ENABLE_IR?=1

####################################################
# Use ENABLE_FINDME=1 to enable Find Me profile support
ENABLE_FINDME?=1

#
# App defines
#
CY_APP_DEFINES += \
  -DSUPPORT_KEY_REPORT \
  -DSWITCH_DIRECT_TO_UNDIRECT_ADV \
  -DSFI_DEEP_SLEEP \
  -DBATTERY_REPORT_SUPPORT \
  -DWICED_BT_TRACE_ENABLE \
  -DLED_SUPPORT=$(LED) \
  -DSLEEP_ALLOWED=$(SLEEP_ALLOWED)

CY_APP_PATCH_LIBS += wiced_hidd_lib.a

ifeq ($(TESTING_USING_HCI),1)
 CY_APP_DEFINES += -DTESTING_USING_HCI
 ifeq ($(ENABLE_BT_SPY),1)
  CY_APP_DEFINES += -DENABLE_BT_SPY_LOG
 endif
endif

ifeq ($(OTA_FW_UPGRADE),1)
 # DEFINES
 CY_APP_DEFINES += -DOTA_FIRMWARE_UPGRADE
 CY_APP_DEFINES += -DDISABLED_PERIPHERAL_LATENCY_ONLY
# OTA_SKIP_CONN_PARAM_UPDATE - When enabled, it will use the current connection parameter for OTA.
#                              When not defined, the OTA libraray will use 7.5 ms interval, makes data transfer much faster.
# CY_APP_DEFINES += -DOTA_SKIP_CONN_PARAM_UPDATE
 ifeq ($(OTA_SEC_FW_UPGRADE), 1)
  CY_APP_DEFINES += -DOTA_SECURE_FIRMWARE_UPGRADE
  COMPONENTS += sec_ota
 else
  COMPONENTS += ota
 endif # OTA_SEC_FW_UPGRADE
 # COMPONENTS
 COMPONENTS += fw_upgrade_lib
else
 ifeq ($(OTA_SEC_FW_UPGRADE),1)
  $(error setting OTA_SEC_FW_UPGRADE=1 requires OTA_FW_UPGRADE also set to 1)
 endif # OTA_SEC_FW_UPGRADE
 COMPONENTS += no_ota
endif # OTA_FW_UPGRADE

ifeq ($(AUDIO),)
 $(info Audio disabled)
else
 # Audio is enabled
 # Check for PDM option
 ifeq ($(PDM),1)
  ifneq ($(filter $(TARGET), CYW920835REF-RCU-01),)
   $(error No PDM available for $(TARGET) -> try TARGET CYW920835M2EVB-01)
  else
   ifneq ($(LED), 0)
     $(info $(TARGET) base board DMIC is shared with LED pins. Enabling DMIC will disable LED)
     LED=0
   endif
   CY_APP_DEFINES += -DSUPPORT_DIGITAL_MIC
   MIC=Digital
  endif
 else
  MIC=Analog
 endif

 #Check for Host interface protocol
 ifeq ($(filter $(AUDIO),IFXV ATV),)
  $(error AUDIO=xxxx, where xxxx must be 'IFXV' or 'ATV')
 else
  # We have supported host interface protocol
  CY_APP_DEFINES += -DSUPPORT_AUDIO -DENABLE_ADC_AUDIO_ENHANCEMENTS
  ifeq ($(AUDIO),IFXV)
   COMPONENTS += ifxv
  else
   COMPONENTS += atv
  endif

  ifeq ($(CODEC),ADPCM)
   COMPONENTS += adpcm
   CY_APP_PATCH_LIBS += adpcm_lib.a
   CY_APP_DEFINES += -DADPCM_ENCODER
  else
   ifeq ($(CODEC),OPUS)
    COMPONENTS += opus
   CY_APP_DEFINES += -DOPUS_ENCODER
    # for *.mk to include CYWxxxxxx_OPUS_CELT.cgs instead of CYWxxxxxx.cgs
    OPUS_CELT_ENCODER = 1
   else
    CY_APP_DEFINES += -DCODEC_STR="Disabled"
    COMPONENTS += pcm
   endif
  endif
 endif
 $(info AUDIO=$(AUDIO), CODEC=$(CODEC), $(MIC) MIC)
endif

ifeq ($(AUTO_RECONNECT),1)
 CY_APP_DEFINES += -DAUTO_RECONNECT
endif

ifeq ($(SKIP_PARAM_UPDATE),1)
 CY_APP_DEFINES += -DSKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
endif

ifeq ($(START_ADV_ON_POWERUP),1)
 CY_APP_DEFINES += -DSTART_ADV_WHEN_POWERUP_NO_CONNECTED
endif

ifeq ($(ENABLE_CONNECTED_ADV),1)
 CY_APP_DEFINES += -DCONNECTED_ADVERTISING_SUPPORTED
endif

ifeq ($(ENDLESS_ADV),1)
 CY_APP_DEFINES += -DENDLESS_LE_ADVERTISING
endif

ifeq ($(LE_LOCAL_PRIVACY),1)
 CY_APP_DEFINES += -DLE_LOCAL_PRIVACY_SUPPORT
endif

ifeq ($(ENABLE_IR),1)
 CY_APP_DEFINES += -DSUPPORT_IR
endif

ifeq ($(ENABLE_FINDME),1)
 CY_APP_DEFINES += -DSUPPORT_FINDME
endif

################################################################################
# Debug
################################################################################
#default trace options
TRACE_BT?=0
TRACE_GATT?=0
TRACE_HIDD?=0
TRACE_HCI?=0
TRACE_HOST?=0
TRACE_SDS?=0
TRACE_LED?=0
TRACE_KEY?=0
TRACE_IR?=0
TRACE_FINDME?=0
TRACE_NVRAM?=0
TRACE_AUDIO?=0
TRACE_MIC?=0
TRACE_CODEC?=0
TRACE_PROTOCOL?=0
TRACE_APP=0

DEFINES += BT_TRACE=$(TRACE_BT) GATT_TRACE=$(TRACE_GATT) HIDD_TRACE=$(TRACE_HIDD) HCI_TRACE=$(TRACE_HCI)
DEFINES += LED_TRACE=$(TRACE_LED) KEY_TRACE=$(TRACE_KEY) NVRAM_TRACE=$(TRACE_NVRAM) HOST_TRACE=$(TRACE_HOST)
DEFINES += AUDIO_TRACE=$(TRACE_AUDIO) MIC_TRACE=$(TRACE_MIC) CODEC_TRACE=$(TRACE_CODEC) PROTOCOL_TRACE=$(TRACE_PROTOCOL)
DEFINES += IR_TRACE=$(TRACE_IR) FINDME_TRACE=$(TRACE_FINDME) SDS_TRACE=$(TRACE_SDS) APP_TRACE=$(TRACE_APP)

################################################################################
# Paths
################################################################################

# Path (absolute or relative) to the project
CY_APP_PATH=.

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI><COMMIT><LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler (Default: GCC in the tools)
CY_COMPILER_PATH=

# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_* \
    $(CY_IDE_TOOLS_DIR))

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

# tools that can be launched with "make open CY_OPEN_TYPE=<tool>
CY_BT_APP_TOOLS=BTSpy ClientControl

-include internal.mk

include $(CY_TOOLS_DIR)/make/start.mk
