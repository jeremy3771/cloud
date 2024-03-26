#!/usr/bin/env python3
import serial
import struct


# DDSM115 MOTOR
class MOTOR_COMMAND:
    def __init__(self, num):
        # Configure the serial port
        self.ser = serial.Serial(
            port='/dev/ttyUSB' + str(num),  # Change this to your actual port
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )   
        self.speed = 0
        self.position = 0
        

    def decimal_to_hex_bytes(self, decimal):
        # 음수를 처리하기 위해 struct 모듈을 사용하여 2의 보수를 취합니다.
        hex_string = format(struct.unpack('>H', struct.pack('>h', decimal))[0], 'x').zfill(4)
        
        # 16진수 문자열을 2바이트로 나누어 각각을 10진수로 변환합니다.
        byte1 = int(hex_string[:2], 16)
        byte2 = int(hex_string[2:], 16)

        # 결과를 반환합니다.
        return byte1, byte2

    def calculate_crc(self, data):
        """Calculate the CRC8/MAXIM of a datagram"""
        CRC8_MAXIM_table = (
            0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
            0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
            0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
            0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
            0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
            0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
            0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
            0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
            0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
            0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
            0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
            0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
            0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
            0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
            0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
            0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
            0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
            0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
            0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
            0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
            0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
            0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
            0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
            0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
            0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
            0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
            0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
            0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
            0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
            0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
            0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
            0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
        )
        crc = 0x00  # Initial CRC value for CRC8/MAXIM

        for c in data:
            crc = CRC8_MAXIM_table[crc ^ c]

        return crc

    def send_data(self,data):
        # Function to send data
        self.ser.write(data)

 

    # 여기부터가 기능 구현 코드
    def ID_SET(self, ID):
        # HEX 형태로 ID 지정
        # ID 지정시에는 하나의 모터만 연결해서 지정
        # ID 지정시에는 모터의 전원을 끊었다가 다시 인가 해서 초기화 후 지정
        data = 0xAA, 0x55, 0x53, ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        for _ in range(5):
            self.send_data(data)

    def ID_QUERY(self):
        # 연결된 하나의 모터 ID값을 불러옵니다
        data = 0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE
        self.send_data(data)
        if self.ser.readable():
            res = self.ser.read(1)
            print("ID: 0x{:02X}".format(ord(res)))

    def SWITCH_VELOCITY_MODE(self, ID):
        # HEX 형태로 ID 지정
        data = ID, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02
        self.send_data(data)
        # ser.close()

    def SWITCH_ANGLE_MODE(self, ID):
        # HEX 형태로 ID 지정
        data = ID, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03
        self.send_data(data)
        # ser.close()

    def SET_VELOCITY(self, ID,SPEED, ACC = 0 ):
        # ID는 HEX, SPEED는 DEC로 지정
        speed_H,speed_L = self.decimal_to_hex_bytes(SPEED)
        data_temp = ID, 0x64, speed_H, speed_L, 0x00, 0x00, 0x00, 0x00, 0x00
        crc = self.calculate_crc(data_temp)
        data = ID, 0x64, speed_H, speed_L, 0x00, 0x00, ACC, 0x00, 0x00, crc
        self.send_data(data)
        # ser.close()

    def SET_ANGLE(self, ID, ANGLE):
        # ID는 HEX, ANGLE는 DEC로 지정
        # 91 = 1degree
        ANGLE = int(ANGLE *(32767 / 360))
        ANGLE_H,ANGLE_L = self.decimal_to_hex_bytes(ANGLE)
        data_temp = ID, 0x64, ANGLE_H, ANGLE_L, 0x00, 0x00, 0x00, 0x00, 0x00
        crc = self.calculate_crc(data_temp)
        data = ID, 0x64, ANGLE_H, ANGLE_L, 0x00, 0x00, 0x00, 0x00, 0x00, crc
        self.send_data(data)
        # ser.close()

    def BRAKE(self, ID):
        # HEX 형태로 ID 지정
        data_temp = ID, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00
        crc = self.calculate_crc(data_temp)
        data = ID, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, crc
        self.send_data(data)
        # ser.close()