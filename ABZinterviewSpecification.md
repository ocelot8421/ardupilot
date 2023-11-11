A firmware (https://github.com/ArduPilot/ardupilot) küldjön vissza másodpercenként 1 üzenetet, hogy éppen hol tart a missionben, amennyiben nincsen feltöltve rá mission, akkor küldje vissza hogy nincsen mission.
A 4.3.2-es verziójú ArduCopter Tag-en dolgozzon.
Ezen felül hozzon létre egy új Mavlink message-t 1500-as ID-val. A mission indításakor küldje el, úgy, hogy az 1. paraméter értéke 1.

Az ardupilot dokumentációja: https://ardupilot.org/dev/index.html Ezen a weboldalon minden szükséges információ megtalálható.

Vezesse Githubon a fejlesztést. A forráskód és a dokumentáció szükséges.