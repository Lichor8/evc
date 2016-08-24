# import robot subsystems
import rhal.rhal
import detection.detection

# compare strings with numbers
import re

def tryint(x):
    try:
        return int(x)
    except ValueError:
        return x


def splittedname(s):
    return tuple(tryint(x) for x in re.split('([0-9]+)', s))

# create robot objects from classes
rhal = rhal.rhal.Rhal()
det = detection.detection.Detection()

# initialize io
rhal.io()

# check robot io
io_ok = 1
io_read = 1

# main loop (run while robot receives inputs)
while io_ok:

    if io_read:

        # set data to send to arduino
        sdata = '0x1|y1|'       # mov_type = 0 (drive between lines),   xy (goal location) in [m]
        sdata = '1r0.2|'        # mov_type = 1 (turn left),             r (turn radius) in [m]
        sdata = '2d1|'          # mov_type = 2 (drive),                 d (distance) in [m]
        sdata = '3r0.2|'        # mov_type = 3 (turn right),            r (turn radius) in [m]
        sdata = '4a180|'        # mov_type = 4 (turn),                  a (angle) in [deg]
        sdata = '5t1|'          # mov_type = 5 (stop),                  t (time) in [s]

        rhal.setsdata(sdata)

        # trigger coordinator/composer of rhal
        rhal.rhal()
        rdata = rhal.get_rdata()
        # print(rdata)

        rdata_split = splittedname(rdata)
        # print(rdata_split)
        test = "('done', 5)"
        print(test)

        if rdata_split == test:
            print(rdata)

        if set(rdata.split()) == set('done5'):
            print(rdata)

        # trigger coordinator/composer of detection
        # det.detection()
        # gappos = det.getgappos()
        # print("correct gapposition",gappos)

        # stop main loop
        # break
