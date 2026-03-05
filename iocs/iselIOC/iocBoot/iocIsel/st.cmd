#!../../bin/windows-x64-static/isel

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/isel.dbd"
isel_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## 
< motor.cmd.iMC

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("isel:")

# Boot complete
