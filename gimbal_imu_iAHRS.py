import time
from dynamixel_sdk import *  # 다이나믹셀 SDK

# 시리얼 및 다이나믹셀 설정 상수
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_SPEED = 115200
DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DXL_IDS = [10, 11, 12, 13]
DXL_INITIAL_POSITION = 2048

# 각 모터별 가동 범위
DXL_RANGE = {
    10: (1703, 2420),
    11: (1682, 2355),
    12: (1797, 2300),
    13: (1777, 2283)
}

POSITION_FACTOR = 11.37
PROFILE_VELOCITY = 100
PROFILE_ACCELERATION = 20

# 포트 핸들러 및 패킷 핸들러 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# 그룹 동기 쓰기/읽기 설정
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 116, 4)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, 132, 4)

class IMU_Dynamixel_Controller:
    def __init__(self):
        self.serial = self.serial_open()
        self.max_data = 10

        # 다이나믹셀 초기화
        self.initialize_port()
        self.setup_dynamixel_motors()

    # 시리얼 포트 열기
    def serial_open(self):
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_SPEED, timeout=0.1)
            print(f"{SERIAL_PORT} open success")
            return ser
        except serial.SerialException as e:
            print(f"Error: unable to open {SERIAL_PORT}: {e}")
            return None

    # 포트 열기 및 보드레이트 설정
    def initialize_port(self):
        if not portHandler.openPort():
            raise Exception("포트 열기에 실패했습니다.")
        if not portHandler.setBaudRate(BAUDRATE):
            raise Exception("보드레이트 설정에 실패했습니다.")

    # 다이나믹셀 모터 설정
    def setup_dynamixel_motors(self):
        for id in DXL_IDS:
            self.set_drive_mode(id, 0)
            self.enable_dynamixel_torque(id)
            self.set_dynamixel_profile_velocity(id, PROFILE_VELOCITY)
            self.set_dynamixel_profile_acceleration(id, PROFILE_ACCELERATION)
            self.move_dynamixel_to_position(id, DXL_INITIAL_POSITION)

    # 다이나믹셀 관련 함수들
    def set_drive_mode(self, dxl_id, mode):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, 10, mode)
        self.check_comm_result(dxl_comm_result, dxl_error, f"Drive Mode 설정 실패: {dxl_id}")

    def enable_dynamixel_torque(self, dxl_id):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, 64, 1)
        self.check_comm_result(dxl_comm_result, dxl_error, f"Torque Enable 실패: {dxl_id}")
        groupSyncRead.addParam(dxl_id)

    def disable_dynamixel_torque(self, dxl_id):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, 64, 0)
        self.check_comm_result(dxl_comm_result, dxl_error, f"Torque Disable 실패: {dxl_id}")

    def set_dynamixel_profile_velocity(self, dxl_id, velocity):
        packetHandler.write4ByteTxRx(portHandler, dxl_id, 112, velocity)

    def set_dynamixel_profile_acceleration(self, dxl_id, acceleration):
        packetHandler.write4ByteTxRx(portHandler, dxl_id, 108, acceleration)

    def move_dynamixel_to_position(self, dxl_id, position):
        position = round(position)
        param_goal_position = [
            DXL_LOBYTE(DXL_LOWORD(position)),
            DXL_HIBYTE(DXL_LOWORD(position)),
            DXL_LOBYTE(DXL_HIWORD(position)),
            DXL_HIBYTE(DXL_HIWORD(position))
        ]
        groupSyncWrite.addParam(dxl_id, param_goal_position)
        groupSyncWrite.txPacket()
        groupSyncWrite.clearParam()

    # 통신 결과 확인 함수
    def check_comm_result(self, dxl_comm_result, dxl_error, error_msg):
        if dxl_comm_result != COMM_SUCCESS:
            print(f"{error_msg}, {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Error: {packetHandler.getRxPacketError(dxl_error)}")

    # 시리얼 데이터를 통해 IMU 데이터 수신
    def send_recv(self, command, data_length):
        if not self.serial:
            return 0, []

        self.serial.write(command.encode())
        recv_buff = self.serial.readline().decode().strip()

        if recv_buff and command[:-1] == recv_buff[:len(command)-1] and recv_buff[len(command)-1] == '=':
            data_str = recv_buff[len(command):]
            data_list = data_str.split(',')
            data_count = min(len(data_list), data_length)
            try:
                returned_data = [int(data, 16) if data.startswith('0x') else float(data) for data in data_list[:data_count]]
                return data_count, returned_data
            except ValueError:
                pass
        return 0, []

    # IMU 데이터 읽기 및 다이나믹셀 제어
    def read_and_control(self):
        data_count, data = self.send_recv("e\n", self.max_data)
        if data_count >= 3:
            roll, pitch, yaw = data[:3]
            print(f"Euler angles - Roll: {roll}, Pitch: {pitch}")
            self.control_dynamixel(pitch, roll)
        else:
            print("Received incomplete Euler angles data")

    # 다이나믹셀 제어 함수
    def control_dynamixel(self, ang_pitch, ang_roll):
        if abs(ang_pitch) > 3:
            move_value_pitch = -ang_pitch * POSITION_FACTOR
            self.control_pitch(move_value_pitch)
        else:
            self.control_pitch(0)

        if abs(ang_roll) > 3:
            move_value_roll = ang_roll * POSITION_FACTOR
            self.control_roll(move_value_roll)
        else:
            self.control_roll(0)

    def control_pitch(self, move_value_pitch):
        target_position_10, target_position_11 = self.calculate_target_position(10, 11, move_value_pitch)
        if target_position_10 is not None:
            self.move_dynamixel_to_position(10, target_position_10)
            self.move_dynamixel_to_position(11, target_position_11)

    def control_roll(self, move_value_roll):
        target_position_12, target_position_13 = self.calculate_target_position(12, 13, move_value_roll)
        if target_position_12 is not None:
            self.move_dynamixel_to_position(12, target_position_12)
            self.move_dynamixel_to_position(13, target_position_13)

    # 목표 위치 계산
    def calculate_target_position(self, id_1, id_2, move_value):
        target_position_1 = DXL_INITIAL_POSITION + move_value
        target_position_2 = DXL_INITIAL_POSITION - move_value

        range_1_min, range_1_max = DXL_RANGE[id_1]
        range_2_min, range_2_max = DXL_RANGE[id_2]

        if range_1_min <= target_position_1 <= range_1_max and range_2_min <= target_position_2 <= range_2_max:
            return target_position_1, target_position_2
        return None, None

    # 종료 처리
    def cleanup(self):
        for id in DXL_IDS:
            self.disable_dynamixel_torque(id)

# 메인 함수
def main():
    controller = IMU_Dynamixel_Controller()

    try:
        while True:
            controller.read_and_control()  # IMU 데이터 읽기 및 제어
            time.sleep(0.01)  # 1Hz 주기로 실행
    except KeyboardInterrupt:
        print("프로그램이 중단되었습니다. 모든 모터의 토크를 비활성화합니다.")
    finally:
        controller.cleanup()
        portHandler.closePort()

if __name__ == "__main__":
    main()
