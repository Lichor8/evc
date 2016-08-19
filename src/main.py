# import robot subsystems
import rhal.rhal
import detection.detection

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
        sdata = '1x1|y1|'
        # sdata = '0x0|y1|'       # stop arduino
        rhal.setsdata(sdata)

        # trigger coordinator/composer of rhal
        rhal.rhal()
        rdata = rhal.get_rdata()
        print(rdata)

        # trigger coordinator/composer of detection
        # det.detection()
        # gappos = det.getgappos()
        # print("correct gapposition",gappos)

        # stop main loop
        # break
