import serial  # 导入模块
import buma
from pyautogui import press

def serial_open():
    try:
        
        portx = "COM4"
        # 50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
        bps = 115200
        #  
        timex = 5
        # 
        ser = serial.Serial(portx, bps, timeout=timex)
        return ser
    except Exception as e:
        print("---异常---：", e)


def watch_serial():
        ser = serial_open()

        message_period = 6
        mark_bytes_num = 4

        
        caught = False
        for j in range(10000):
            if not caught:
                if ser.read().hex() == '01':
                    if ser.read().hex() == 'fe':
                        if ser.read().hex() == '02':
                            if ser.read().hex() == 'ff':
                                caught = True
            else:
                for i in range(10000):
                    xyz_read = dict()
                    for axis in ['x', ' y', 'z']:
                        hex_read_1 = ser.read().hex()
                        hex_read_0 = ser.read().hex()
                        int_read_1 = int(hex_read_1, 16)
                        int_read_0 = int(hex_read_0, 16)
                        full_int_read = int_read_1 * 256 + int_read_0
                        xyz_read[axis] = buma.buma_value(full_int_read, 16)
                    print(xyz_read)
                    for k in range(mark_bytes_num):
                        _ = ser.read().hex()
                    # if full_int_read < 0:
                    #     print(0)
                    # else:
                    #     print(1)

            # print(hex_read)  # 读一个字节
            # if hex_read=='aa':
            #     print('you are me')
            #     press('up')
            # print(int(hex_read,16))  # 读一个字节
            # print(type(int(hex_read,16)))
        ser.close()  # 关闭串口

def get_avg_stop_point(ser):
    # ser = serial_open()

    message_period = 6
    mark_bytes_num = 4

    
    
    caught = False
    for j in range(10000):
        if not caught:
            if ser.read().hex() == '01':
                if ser.read().hex() == 'fe':
                    if ser.read().hex() == '02':
                        if ser.read().hex() == 'ff':
                            caught = True
        else:
            todo_avg_x = []
            todo_avg_y = []
            for i in range(10):
                xyz_read = dict()
                for axis in ['x', 'y', 'z']:
                    hex_read_1 = ser.read().hex()
                    hex_read_0 = ser.read().hex()
                    int_read_1 = int(hex_read_1, 16)
                    int_read_0 = int(hex_read_0, 16)
                    full_int_read = int_read_1 * 256 + int_read_0
                    xyz_read[axis] = buma.buma_value(full_int_read, 16)
                todo_avg_x.append(xyz_read['z'])
                todo_avg_y.append(xyz_read['y'])

                for k in range(mark_bytes_num):
                    _ = ser.read().hex()
            avg_xy = (sum(todo_avg_x) // 10, sum(todo_avg_y) // 10)
            print(len(todo_avg_x), len(todo_avg_y))
            print(avg_xy)
            # ser.close()  # 关闭串口
            return avg_xy
                


def read_one_period(ser):
    try:
        

        message_period = 10
        mark_bytes_num = 4

        
        caught = False
        for j in range(20):
            if not caught:
                if ser.read().hex() == '01':
                    if ser.read().hex() == 'fe':
                        if ser.read().hex() == '02':
                            if ser.read().hex() == 'ff':
                                caught = True
            else:
                # for i in range(10000):
                xyz_read = dict()
                for axis in ['x', ' y', 'z']:
                    hex_read_1 = ser.read().hex()
                    hex_read_0 = ser.read().hex()
                    int_read_1 = int(hex_read_1, 16)
                    int_read_0 = int(hex_read_0, 16)
                    full_int_read = int_read_1 * 256 + int_read_0
                    xyz_read[axis] = buma.buma_value(full_int_read, 16)
                # print(xyz_read)
                for k in range(mark_bytes_num):
                    _ = ser.read().hex()
                return xyz_read
                

    #
    except Exception as e:
        print("---异常---：", e)

if __name__ == '__main__':
    watch_serial()

