def rpi2arduino(ser, sdata):
    ser.write(sdata)
    ser.write('0x1127|y135|')
    # ser.write('4d180|')
    # ser.write('5t8|')
    print(ser.readline())

    # if ser.inWaiting() > 0:
    # bytesToRead = ser.inWaiting()
    # print(ser.inWaiting())
    # print(ser.read(bytesToRead))

    # time.sleep(1)


def arduino2rpi(ser):
    rdata = print(ser.readline())

    return rdata
