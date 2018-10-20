@echo Make sure phone is plugged in
pause
@echo Running
cd ADB
adb tcpip 5555
adb connect 192.168.49.1:5555
@echo You can now disconnect the phone
pause