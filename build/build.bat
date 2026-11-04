@echo off
rem Rebuild the Vivado project from src/ and generate the bitstream (Windows).
rem Requires Vivado 2022.2. If it is not at the default path, set VIVADO_BAT first, e.g.
rem   set VIVADO_BAT=D:\apps\Vivado\2022.2\bin\vivado.bat
setlocal
if "%VIVADO_BAT%"=="" set VIVADO_BAT=D:\apps\Vivado\2022.2\bin\vivado.bat
if not exist "%VIVADO_BAT%" (
  echo ERROR: vivado.bat not found: %VIVADO_BAT%
  exit /b 1
)
cd /d "%~dp0.."
call "%VIVADO_BAT%" -mode batch -nojournal -log build\create_project.log -source build\create_project.tcl || exit /b 1
call "%VIVADO_BAT%" -mode batch -nojournal -log build\build_bitstream.log -source build\build_bitstream.tcl || exit /b 1
echo Output: build\output\robot_system_wrapper.bit and .hwh
endlocal
