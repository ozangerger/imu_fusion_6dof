import serial
import time
import csv


def main():
    with open('sensor_data.csv', mode='w') as csv_file:
        sensor_writer = csv.writer(csv_file, delimiter=',')
        sensor_writer.writerow(["time", "accX", "accY", "accZ", "gyX", "gyY", "gyZ",
                                "ahrs_pitch", "ahrs_roll", "ahrs_yaw", "cf_pitch", "cf_roll"])

    com = "/dev/cu.usbmodem1101"
    baud = 500000

    x = serial.Serial(com, baud, timeout=0.1)

    while x.is_open:
        data = str(x.readline().decode('utf-8')).rstrip()
        if data is not '':
            print(data)
            with open('sensor_data.csv', mode='a') as csv_file:
                sensor_writer = csv.writer(csv_file, quoting=csv.QUOTE_NONE)
                sensor_writer.writerow(data.split(','))

if __name__ == '__main__':
    main()
