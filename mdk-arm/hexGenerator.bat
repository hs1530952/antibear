@echo off

:::::::: get git commit hash ::::::::
for /F "tokens=* USEBACKQ" %%a in (`git log -1 --format^="%%h"`) do (set revision=%%a)
:::::::::::::::::::::::::::::::::::::

copy ..\obj\mdk-arm\main\antiBear.hex ..\obj\antiBear_%revision%.hex
