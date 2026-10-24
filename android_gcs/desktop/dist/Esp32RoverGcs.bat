@echo off
rem ESP32 Rover GCS for Windows. Needs Java 17 or newer (e.g. Eclipse Temurin 17 JRE).
where javaw >nul 2>nul
if errorlevel 1 (
  echo Java was not found. Install Java 17 from https://adoptium.net and try again.
  pause
  exit /b 1
)
start "" javaw -jar "%~dp0Esp32RoverGcs.jar"
