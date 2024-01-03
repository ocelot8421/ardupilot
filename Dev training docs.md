WINDOWS ELÉRÉSI ÚTVONAL

C:\Users\Euronics\AppData\Local\Packages\CanonicalGroupLimited.Ubuntu_79rhkp1fndgsc\LocalState\rootfs\home\hajni

cd ../../../../home/hajni/ardupilotcd
./waf configure --board sitl --debug

nice ../Tools/autotest/sim_vehicle.py --map --console -D -G

---

Debug

Permission denied: '/home/hajni/ardupilot/.vscode/c_cpp_properties.json
---> törölni a .vscode mappát, majd újra futatni a:
../Tools/autotest/sim_vehicle.py --map --console -D -G

wp load ../ArduCopter/ABZ_mission_test/testway_laptop.txt

---

https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

Tools/environment_install/install-prereqs-ubuntu.sh -y
