# import robot subsystems
import detection.detection

# create robot objects from classes
det = detection.detection.Detection()

# check robot inputs
io_ok = 1
io_read = 1

# main loop (run while robot receives inputs)
while io_ok:

    if io_read:

        # trigger coordinator/composer of detection
        det.detection()
        gappos = det.getgappos()
        print("correct gapposition",gappos)

        # stop main loop
        break
