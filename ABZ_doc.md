# ABZ
## Specification
A firmware (https://github.com/ArduPilot/ardupilot) küldjön vissza másodpercenként 1 üzenetet, hogy éppen hol tart a missionben, amennyiben nincsen feltöltve rá mission, akkor küldje vissza hogy nincsen mission.
A 4.3.2-es verziójú ArduCopter Tag-en dolgozzon.
Ezen felül hozzon létre egy új Mavlink message-t 1500-as ID-val. A mission indításakor küldje el, úgy, hogy az 1. paraméter értéke 1.

Az ardupilot dokumentációja: https://ardupilot.org/dev/index.html Ezen a weboldalon minden szükséges információ megtalálható.

## Mission feedback in every second
### Copter::mission_feedback()
This function is called by the Copter class in the one_hz_loop(). To obtain the current state of the mission, we use the get_current_nav_cmd() function of AP_Mission.cpp. The value of the current mission command ID indicates which mission command is currently running, obtained by calling AP_Mission::type(). If the ID is zero (null), the switch function of type() would trigger AP_HAL::panic(). In such cases, a simple text ("There is no mission") will be printed. It is important to note that the no-mission text is displayed only when there is no mission stored. Therefore, if a mission is loaded but not yet completed (e.g., if we switch to guided mode during the mission process), the current state of the actual mission will still be printed.


## Custom mavlink message
### Creating new mavlink message
To create a new MAVLink message, it must be implemented in ardupilotmega.xml (located in the mavlink submodule). We can only push to the forked repository: [https://github.com/ocelot8421/mavlink/tree/ABZinterview_mavlink_message]. The MSG_ABZ_ID1500 command handles param 1 through ABZ_ENUM::ABZ_ENTRY, stored in the abz_enum field (which also needs to be created in the XML). To obtain new MAVLink packet codes (mavlink_msg_abz_id1500.h and the regenerated ardupilotmega.h), the entire codebase must be rebuilt using the command: `./waf copter`.
### Sending new mavlink message
The "waiting_to_start" flag indicates whether a mission has already started. It is declared in mode.h and defined in mode_auto::init() initially. In the run() function, if waiting_to_start is true, we can be sure that we are before the mission and can call send_message(MSG_ABZ_ID1500). Ensure to add the message into the enum ap_message and include it as a switch case condition in GCS_MAVLINK_Copter::try_send_message(enum ap_message id).

## Note
\* Make sure that the repository is in the same filesystem as WSL (if used) for optimal build performance. <br>
The WSL path on Win10 is: <br>
C:\Users\USERNAME_WIN10\AppData\Local\Packages\CanonicalGroupLimited.Ubuntu_IDENTICAL_STRING\LocalState\rootfs\home\USERNAMEWSL

