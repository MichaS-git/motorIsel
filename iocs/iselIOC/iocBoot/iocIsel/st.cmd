#!../../bin/linux-x86_64/isel

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

# Boot complete
