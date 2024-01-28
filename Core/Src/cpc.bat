set SCRIPT_DIR=%~dp0%
cd %SCRIPT_DIR%
copy main.cpp main.c
del main.cpp
