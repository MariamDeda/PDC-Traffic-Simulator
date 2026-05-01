@echo off
echo ========================================
echo Building Message Batching Test
echo Milestone 3
echo ========================================

echo Compiling...

g++ -std=c++17 test_batching.cpp DecentralizedController.cpp TrafficGenerator.cpp -o test_batching.exe

if %errorlevel% equ 0 (
    echo.
    echo Compilation successful!
    echo.
    echo Running test...
    echo.
    test_batching.exe
) else (
    echo.
    echo Compilation failed!
    pause
)