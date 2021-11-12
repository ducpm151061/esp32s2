#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := template

EXTRA_COMPONENT_DIRS += $(IDF_PATH)/esp-idf-lib/components
EXCLUDE_COMPONENTS := aht led_strip

include $(IDF_PATH)/make/project.mk
