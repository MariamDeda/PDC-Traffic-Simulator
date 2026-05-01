@echo off
echo Building Simple Deadline Test...

g++ -std=c++17 test_deadline.cpp DecentralizedController.cpp TrafficGenerator.cpp -o test_deadline.exe

if %errorlevel% equ 0 (
    echo.
    echo ========================================
    echo COMPILATION SUCCESSFUL!
    echo ========================================
    echo.
    test_deadline.exe
) else (
    echo.
    echo ========================================
    echo COMPILATION FAILED!
    echo ========================================
    echo.
    pause
)