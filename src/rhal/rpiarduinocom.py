def rpi2arduino(ser, sdata):
    # sdata = '0x1127|y135|'
    # print(sdata.encode('UTF-8'))
    ser.write(sdata.encode('UTF-8'))
    # ser.write('0x1127|y135|')
    # ser.write('4d180|')
    # ser.write('5t8|')
    # print(ser.readline().decode('utf-8'))


    # if ser.inWaiting() > 0:
    # bytesToRead = ser.inWaiting()
    # print(ser.inWaiting())
    # print(ser.read(bytesToRead))

    # time.sleep(1)


def arduino2rpi(ser):
    # rdata = print(ser.readline().decode('utf-8'))
    rdata = ser.readline().decode('utf-8')

    return rdata
