from time import strftime
import serial
import sys
import datetime

def main():
    if len(sys.argv) != 2:
        exit(f"Usage: python offline_calibration.py <PORT>\nExample: python3 offline_calibration.py COM3")

    port = sys.argv[1];
    # Serial connection
    serialConn = serial.Serial(port, 115200)
    print("Running...");

    now = datetime.datetime.now()
    with open(f'calib_{now.strftime("%Y%m%d-%H%M%S")}.csv', 'w') as f:
        f.write("imu,it,ex,ey,ez,ax,ay,az\n")
        data_dec = ''
        while True:
            data = serialConn.readline()
            data_dec = data.decode("UTF-8").rstrip() + '\n'
            if data_dec[0] == "{":
                break

            print(data_dec)
            f.write(data_dec)

    serialConn.close()

if __name__ == '__main__':
    main()
