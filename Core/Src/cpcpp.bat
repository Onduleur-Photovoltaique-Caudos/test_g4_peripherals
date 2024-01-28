set SCRIPT_DIR=%~dp0%
cd %SCRIPT_DIR%
copy main.c main.cpp
del main.c
