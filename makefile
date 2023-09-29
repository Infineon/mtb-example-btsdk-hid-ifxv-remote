#
# Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
#
# This software, including source code, documentation and related
# materials ("Software") is owned by Cypress Semiconductor Corporation
# or one of its affiliates ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products.  Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif

#
# Basic Configuration
#
APPNAME=BLE_IFXV_Remote
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=

# default target
TARGET=CYW920835M2EVB-01

SUPPORTED_TARGETS = \
    CYW920835REF-RCU-01 \
    CYW920835M2EVB-01

ifeq ($(filter $(TARGET),$(SUPPORTED_TARGETS)),)
 $(error TARGET $(TARGET) is not supported for this application. Edit SUPPORTED_TARGETS in the code example makefile to add new BSPs)
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
COMPONENTS += bsp_design_modus
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
TESTING_USING_HCI?=1

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
LED?=1

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
#      -            leave it empty or not to define to disable codec (Data will be sent as PCM raw data. This option is not capable for live audio streaming.)
#      - OPUS       use OPUS CELT encoder
#      - ADPCM      use ADPCM encoder
#
# PDM=x
#    - Use this option to enable/disable digital microphone (PDM=1/0). The AUDIO option must be enabled for this option to take effect.
#    - This option is invalid for CYW920835REF-RCU-01 BSP as it does not PDM hardware. In CYW920835M2EVB-01 platform,
#      since the GPIO pins are shared with LEDs, make sure SW4 is switched to DMIC and the LED is disabled for the complier option.
#
AUDIO?=IFXV
CODEC?=OPUS
PDM?=0

#
# App defines
#
CY_APP_DEFINES = \
  -DSUPPORT_KEY_REPORT \
  -DBT_CONFIGURATOR_SUPPORT \
  -DAUTO_PAIRING \
  -DBATTERY_REPORT_SUPPORT \
  -DBLE_SUPPORT \
  -DWICED_BT_TRACE_ENABLE \
  -DLED_SUPPORT=$(LED) \
  -DSLEEP_ALLOWED=$(SLEEP_ALLOWED)

CY_APP_PATCH_LIBS += wiced_hidd_lib.a

ifeq ($(TESTING_USING_HCI),1)
 CY_APP_DEFINES += -DTESTING_USING_HCI
endif

ifeq ($(filter $(TARGET), CYW920835REF-RCU-01),)
 CY_APP_DEFINES += -DWICED_EVAL
else
 CY_APP_DEFINES += -DSUPPORT_KEYSCAN
endif

ifeq ($(AUDIO),)
 $(info Audio disabled)
else
 # Audio is enabled
 # Check for PDM option
 ifeq ($(PDM),1)
  ifneq ($(filter $(TARGET), CYW920835REF-RCU-01),)
   $(error Cannot enable PDM for $(TARGET))
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
   $(error Google Voice is not implemeneted for this release)
  endif

  ifeq ($(CODEC),MSBC)
    COMPONENTS += msbc
  else
   ifeq ($(CODEC),ADPCM)
    COMPONENTS += adpcm
    CY_APP_PATCH_LIBS += adpcm_lib.a
    CY_APP_DEFINES += -DADPCM_ENCODER
   else
    ifeq ($(CODEC),OPUS)
     COMPONENTS += opus
     # for *.mk to include CYWxxxxxx_OPUS_CELT.cgs instead of CYWxxxxxx.cgs
     OPUS_CELT_ENCODER = 1
    else
     COMPONENTS += pcm
    endif
   endif
  endif
 endif
 $(info AUDIO=$(AUDIO), CODEC=$(CODEC), $(MIC) MIC)
endif

################################################################################
# Debug
################################################################################
#default trace options
TRACE_BT?=0
TRACE_GATT?=0
TRACE_HIDD?=0
TRACE_HCI?=0
TRACE_LED?=0
TRACE_KEY?=0
TRACE_NVRAM?=0
TRACE_AUDIO?=0
TRACE_MIC?=0
TRACE_CODEC?=0
TRACE_PROTOCOL?=0

DEFINES += BT_TRACE=$(TRACE_BT) GATT_TRACE=$(TRACE_GATT) HIDD_TRACE=$(TRACE_HIDD) HCI_TRACE=$(TRACE_HCI)
DEFINES += LED_TRACE=$(TRACE_LED) KEY_TRACE=$(TRACE_KEY) NVRAM_TRACE=$(TRACE_NVRAM)
DEFINES += AUDIO_TRACE=$(TRACE_AUDIO) MIC_TRACE=$(TRACE_MIC) CODEC_TRACE=$(TRACE_CODEC) PROTOCOL_TRACE=$(TRACE_PROTOCOL)

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
