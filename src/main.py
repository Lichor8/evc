# import robot subsystems
import rhal.rhal
import detection.detection
import strategy.strategy
import strategy.following_strategy

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
strat = strategy.strategy.Strategy()
fol = strategy.following_strategy.Follow()


def tryint(x):
    try:
        return int(x)
    except ValueError:
        return x


def splittedname(s):
    return tuple(tryint(x) for x in re.split('([0-9]+)', s))

# initialize io
#rhal.io()

# check robot io
io_ok = 1
io_read = 1
challenge = 1
ini = 1

# main loop (run while robot receives inputs)
while io_ok:

    if io_read:

        if challenge:       # challenge mode
            # trigger coordinator/composer of rhal
            rhal.rhal()
            rdata = rhal.get_rdata()
            print(rdata)

            # split and compare two strings to enable if statement
            rdata_split = splittedname(rdata)
            do_strat_if = [splittedname('done1\r\n'), splittedname('done2\r\n'), splittedname('done3\r\n'),
                           splittedname('done4\r\n'), splittedname('done5\r\n'), splittedname('execute0\r\n')]
            # execute_split = [splittedname('execute1\r\n'), splittedname('execute2\r\n'), splittedname('execute3\r\n'),
            #                  splittedname('execute4\r\n'), splittedname('execute5\r\n')]

            if rdata_split in do_strat_if or ini:
                strat.strategy(rhal)
                ini = 0

        else:               # following mode
            fol.follow()

        # set data to send to arduino
        # sdata = '0x0|y1|'       # mov_type = 0 (drive between lines),   xy (goal location) in [m]
        # sdata = '1r0.2|'        # mov_type = 1 (turn left),             r (turn radius) in [m]
        # sdata = '2d1|'          # mov_type = 2 (drive),                 d (distance) in [m]
        # sdata = '3r0.2|'        # mov_type = 3 (turn right),            r (turn radius) in [m]
        # sdata = '4a180|'        # mov_type = 4 (turn),                  a (angle) in [deg]
        # sdata = '5t5|'          # mov_type = 5 (stop),                  t (time) in [s]

        # rhal.setsdata(sdata)

        # trigger coordinator/composer of rhal
        # rhal.rhal()
        # rdata = rhal.get_rdata()
        # print(rdata)

        # stop main loop
        # break
