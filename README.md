## serial comunication with vex brain

### vex brain side
request:
0b
b0 - odometry (x, y, h) in standard units (mm, rad)
b1 - heading only (in radians)
b2 - dataPack 1
b3 - dataPack 2
b4 - dataPack 4
b5 - dataPack 5
b6 - dataPack 6
b7 - dataPack 7

### external coprocesor side

odometry data (3 + 2 bytes)

x in -2000, 2000 (12 bits)
y in -2000, 2000 (12 bits)
h in 0, 2pi
encoding: eh = (uint16) (h * 65536 / 2pi + 0.5f)
decoding: h = (float)(int16)eh * 2pi / 65536

