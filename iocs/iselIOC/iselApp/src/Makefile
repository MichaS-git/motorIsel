TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = isel
# owis.dbd will be created and installed
DBD += isel.dbd

# owis.dbd will be made up from these files:
isel_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
isel_DBD += asyn.dbd
isel_DBD += drvAsynSerialPort.dbd
#endif
isel_DBD += motorSupport.dbd
isel_DBD += IselSupport.dbd

# Add all the support libraries needed by this IOC
isel_LIBS += Isel
isel_LIBS += motor
#ifdef ASYN
isel_LIBS += asyn
#endif

# owis_registerRecordDeviceDriver.cpp derives from owis.dbd
isel_SRCS += isel_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
isel_SRCS_DEFAULT += iselMain.cpp
isel_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#owis_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
isel_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

