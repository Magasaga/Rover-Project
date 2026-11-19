@echo off
title Push Eco-Eco Rover to GitHub
echo ========================================================
echo   Eco-Eco Rover: Push to GitHub Helper
echo ========================================================
echo.

where git >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo [!] Git is not currently installed on your computer.
    echo.
    echo You have 2 easy options:
    echo 1. Install Git from: https://git-scm.com/download/win
    echo    (Then double-click this script again)
    echo.
    echo 2. OR Upload directly via browser:
    echo    Open your repo on github.com -^> Click 'Add file' -^> 'Upload files'
    echo    and drag the files into the browser.
    echo.
    pause
    exit /b
)

echo Repository URL format: https://github.com/Magasaga/<repo-name>.git
set /p REPO_URL="Enter your full GitHub repository URL: "

if "%REPO_URL%"=="" (
    echo [ERROR] No repository URL entered. Exiting.
    pause
    exit /b
)

echo.
echo [1/4] Initializing Git...
git init

echo [2/4] Staging files...
git add index.html README.md Rover_ESP32_Drive/

echo [3/4] Creating commit...
git commit -m "Add ESP32-S3 4-pin BTS7960 rover drive system and dashboard"

echo [4/4] Pushing to %REPO_URL%...
git branch -M main
git remote remove origin >nul 2>&1
git remote add origin %REPO_URL%
git push -u origin main

echo.
echo ========================================================
echo Process completed! Check your repository on GitHub.
echo ========================================================
pause
