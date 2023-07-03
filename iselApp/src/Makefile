# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += IselSupport.dbd

LIBRARY_IOC = Isel

# Step-Controller iMC-XX
SRCS += iMCDriver.cpp

Isel_LIBS += motor asyn
Isel_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

