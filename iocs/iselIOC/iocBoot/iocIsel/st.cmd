#!../../bin/windows-x64-static/isel

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/isel.dbd"
isel_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=isel:")

## 
< motor.cmd.iMC

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("isel:")

# we don't want any retrys on the axes
dbpf isel:m1.RTRY 0
dbpf isel:m2.RTRY 0
dbpf isel:m3.RTRY 0
dbpf isel:m4.RTRY 0

# Boot complete