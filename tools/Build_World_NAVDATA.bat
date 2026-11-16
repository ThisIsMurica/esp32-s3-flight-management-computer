@echo off
setlocal
cd /d "%~dp0"

echo MINI AVIONICS - BUILD WORLD NAVDATA
echo ===================================
echo.

where py >nul 2>nul
if %errorlevel%==0 (
    py -3 build_world_hybrid_navdata.py --faa-package Mini_FMC_NAVDATA_260806.zip --output Mini_FMC_NAVDATA_260806_WORLD.zip
) else (
    where python >nul 2>nul
    if not %errorlevel%==0 (
        echo Python was not found. Install Python 3, then run this file again.
        pause
        exit /b 1
    )
    python build_world_hybrid_navdata.py --faa-package Mini_FMC_NAVDATA_260806.zip --output Mini_FMC_NAVDATA_260806_WORLD.zip
)

if not %errorlevel%==0 (
    echo.
    echo Build failed. Read the error above.
    pause
    exit /b 1
)

echo.
echo DONE.
echo Extract Mini_FMC_NAVDATA_260806_WORLD.zip and copy NAVDATA to the SD-card root.
pause
