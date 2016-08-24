# import robot subsystems
import rhal.rhal
import detection.detection
import strategy.strategy
import strategy.follow_postit

# create robot objects from classes
rhal = rhal.rhal.Rhal()
det = detection.detection.Detection()
strat = strategy.strategy.Strategy()
fol = strategy.follow_postit.Follow()

# initialize io
# rhal.io()

# check robot io
io_ok = 1
io_read = 1

# main loop (run while robot receives inputs)
while io_ok:

    if io_read:

        # set data to send to arduino
        # sdata = '0x117|y135|'
        # rhal.setsdata(sdata)

        # trigger coordinator/composer of rhal
        # rhal.rhal()
        # rdata = rhal.get_rdata()
        # print(rdata)

        # trigger coordinator/composer of detection
        # det.detection()
        # strat.strategy()
        # print(det.gettarget())
        # gappos = det.getgappos()
        # print("correct gapposition",gappos)

        fol.follow()
        # stop main loop
        # break
