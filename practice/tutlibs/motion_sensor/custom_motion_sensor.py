"""
 * @file        custom_motion_sensor.py
 * @brief       Custom motion sensor class for motion sensor data processing and analysis.
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)      
 *
 * @date        2025-08-11 Released by AI Lab, Hanyang University
 * 
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Union
try:
    import rosbag
    use_rosbag = True
    use_rosbags = False
except:
    use_rosbag = False
    try:
        from rosbags.highlevel import AnyReader
        from pathlib import Path
        use_rosbags = True
    except:
        use_rosbags = False

class MotionSensorProcessor:
    """차량 모션 센서 데이터 처리 클래스"""
    
    def __init__(self):
        self.data = {}
        
    def load_rosbag_data(self, bag_path: str, topics: List[str]) -> dict:
        """
        ROS bag 파일에서 지정된 토픽들의 데이터를 로드합니다.
        
        Args:
            bag_path (str): bag 파일 경로
            topics (List[str]): 로드할 토픽 리스트
            
        Returns:
            dict: 토픽별 데이터가 담긴 딕셔너리
        """
        data = {topic: {'time': [], 'data': []} for topic in topics}
        
        try:
            if use_rosbag:
                # Ubuntu/Linux 환경에서 rosbag 사용
                with rosbag.Bag(bag_path, 'r') as bag:
                    for topic, msg, t in bag.read_messages(topics=topics):
                        timestamp = t.to_sec()
                        data[topic]['time'].append(timestamp)
                        data[topic]['data'].append(msg)
            elif use_rosbags:
                # Windows 환경에서 rosbags 라이브러리 사용
                bag_path_obj = Path(bag_path)
                with AnyReader([bag_path_obj]) as reader:
                    # 지정된 토픽들에 대한 연결 필터링
                    connections = [conn for conn in reader.connections if conn.topic in topics]
                    
                    for connection, timestamp, rawdata in reader.messages(connections=connections):
                        # 타임스탬프를 초 단위로 변환 (nanoseconds를 seconds로)
                        timestamp_sec = timestamp / 1e9
                        
                        # 메시지 역직렬화
                        msg = reader.deserialize(rawdata, connection.msgtype)
                        
                        data[connection.topic]['time'].append(timestamp_sec)
                        data[connection.topic]['data'].append(msg)
            else:
                raise ImportError("rosbag 또는 rosbags 라이브러리가 설치되지 않았습니다.")
                    
        except Exception as e:
            print(f"ROS bag 로딩 중 오류 발생: {e}")
            # 시뮬레이션 데이터로 대체
            print("시뮬레이션 데이터를 생성합니다...")
            return self._generate_simulation_data(topics)
            
        return data
    
    def _generate_simulation_data(self, topics: List[str]) -> dict:
        """시뮬레이션 데이터를 생성합니다 (ROS bag이 없을 경우)"""
        data = {topic: {'time': [], 'data': []} for topic in topics}
        
        # 시간 생성 (30초간, 0.1초 간격)
        time_points = np.arange(0, 30, 0.1)
        
        for topic in topics:
            if '/gps/imu' in topic:
                for t in time_points:
                    # IMU 시뮬레이션 데이터
                    imu_msg = type('IMU', (), {})()
                    imu_msg.angular_velocity = type('AngularVelocity', (), {})()
                    imu_msg.linear_acceleration = type('LinearAcceleration', (), {})()
                    
                    # 원형 운동 시뮬레이션
                    imu_msg.angular_velocity.z = 0.5 * np.sin(0.1 * t) + np.random.normal(0, 0.05)
                    imu_msg.linear_acceleration.x = 2.0 * np.sin(0.2 * t) + np.random.normal(0, 0.1)
                    imu_msg.linear_acceleration.y = 1.5 * np.cos(0.2 * t) + np.random.normal(0, 0.1)
                    
                    data[topic]['time'].append(t)
                    data[topic]['data'].append(imu_msg)
                    
            elif '/novatel/oem7/inspvax' in topic:
                for i, t in enumerate(time_points):
                    # GPS 시뮬레이션 데이터
                    gps_msg = type('GPS', (), {})()
                    
                    # 원형 경로 시뮬레이션
                    radius = 50  # 미터
                    center_lat = 37.5665  # 서울 위도
                    center_lon = 126.9780  # 서울 경도
                    
                    angle = 0.02 * i  # 원형 움직임
                    gps_msg.latitude = center_lat + (radius * np.cos(angle)) / 111320
                    gps_msg.longitude = center_lon + (radius * np.sin(angle)) / (111320 * np.cos(np.radians(center_lat)))
                    
                    data[topic]['time'].append(t)
                    data[topic]['data'].append(gps_msg)
        
        return data
    
    def extract_gps_trajectory(self, gps_data: dict) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        GPS 데이터에서 위도, 경도, 시간 정보를 추출합니다.
        
        Args:
            gps_data (dict): GPS 토픽 데이터
            
        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray]: 위도, 경도, 시간 배열
        """
        latitudes = []
        longitudes = []
        times = []
        
        for time_stamp, msg in zip(gps_data['time'], gps_data['data']):
            latitudes.append(msg.latitude)
            longitudes.append(msg.longitude)
            times.append(time_stamp)
            
        return np.array(latitudes), np.array(longitudes), np.array(times)
    
    def extract_imu_data(self, imu_data: dict) -> dict:
        """
        IMU 데이터에서 각속도와 선형가속도를 추출합니다.
        
        Args:
            imu_data (dict): IMU 토픽 데이터
            
        Returns:
            dict: 추출된 IMU 데이터
        """
        result = {
            'time': [],
            'angular_velocity_z': [],
            'linear_acceleration_x': [],
            'linear_acceleration_y': []
        }
        
        for time_stamp, msg in zip(imu_data['time'], imu_data['data']):
            result['time'].append(time_stamp)
            result['angular_velocity_z'].append(msg.angular_velocity.z)
            result['linear_acceleration_x'].append(msg.linear_acceleration.x)
            result['linear_acceleration_y'].append(msg.linear_acceleration.y)
            
        return result
    
    def average_filter(self, data: np.ndarray) -> np.ndarray:
        """
        평균 필터를 적용합니다.
        
        Args:
            data (np.ndarray): 입력 데이터
            
        Returns:
            np.ndarray: 필터링된 데이터
        """
        # For문을 돌면서 누적된 데이터들의 평균값을 계산하여 반환
        # 초기값은 raw data와 똑같은 값을 반환
        filtered_data = np.zeros_like(data)
        for i in range(len(data)):
            filtered_data[i] = 0 # TODO: 이전까지의 값들을 평균으로 계산하여 반환

        return filtered_data
    
    def moving_average_filter(self, data: np.ndarray, window_size: int = 5) -> np.ndarray:
        """
        이동 평균 필터를 적용합니다.
        
        Args:
            data (np.ndarray): 입력 데이터
            window_size (int): 윈도우 크기
            
        Returns:
            np.ndarray: 필터링된 데이터
        """
        if window_size <= 0:
            return data
        
        filtered_data = np.zeros_like(data)
        
        for i in range(len(data)):
            # TODO: window_size 만큼의 데이터를 평균으로 계산하여 반환
            filtered_data[i] = 0 # TODO
            
        return filtered_data
    
    def exponential_moving_average_filter(self, data: np.ndarray, alpha: float = 0.3) -> np.ndarray:
        """
        지수 이동 평균 필터를 적용합니다.
        
        Args:
            data (np.ndarray): 입력 데이터
            alpha (float): 평활화 계수 (0 < alpha <= 1)
            
        Returns:
            np.ndarray: 필터링된 데이터
        """
        if alpha <= 0 or alpha > 1:
            raise ValueError("alpha 값은 0과 1 사이여야 합니다.")
        
        filtered_data = np.zeros_like(data)
        filtered_data[0] = data[0]
        
        for i in range(1, len(data)):
            filtered_data[i] = 0 # TODO: alpha 값을 고려하여 계산하여 반환
            
        return filtered_data
    
    def generate_wheel_encoder_simulation(self, duration: float = 10.0, 
                                        time_interval: float = 0.001,
                                        wheel_radius: float = 0.6,
                                        pulses_per_revolution: int = 20) -> dict:
        """
        휠 엔코더 시뮬레이션 데이터를 생성합니다.
        
        Args:
            duration (float): 시뮬레이션 시간 (초)
            time_interval (float): 샘플링 간격 (초)
            wheel_radius (float): 휠 반지름 (미터)
            pulses_per_revolution (int): 회전당 펄스 수
            
        Returns:
            dict: 시뮬레이션 데이터
        """
        time_points = np.arange(0, duration, time_interval)
        
        # 연속적인 사인 신호 생성 (0.05Hz)
        gt_velocity = 20 * np.sin(2 * np.pi * 0.05 * time_points)  # m/s
        
        # 위치 계산 (적분)
        position_m = np.cumsum(gt_velocity * time_interval)
        
        # 휠 각도 계산
        wheel_angle_rad = position_m / wheel_radius
        
        # 펄스 사이클에서의 위치
        position_in_pulse_cycle = wheel_angle_rad / (2 * np.pi) * 2 * pulses_per_revolution
        
        # 양자화된 위치
        quant_position_in_pulse_cycle = np.round(position_in_pulse_cycle)
        
        # 펄스 생성 (0 또는 1)
        pulses = np.remainder(quant_position_in_pulse_cycle, 2).astype(int)
        
        return {
            'time': time_points,
            'gt_velocity': gt_velocity,
            'position_m': position_m,
            'wheel_angle_rad': wheel_angle_rad,
            'pulses': pulses,
            'wheel_radius': wheel_radius,
            'pulses_per_revolution': pulses_per_revolution,
            'time_interval': time_interval
        }
    
    def pulse_counting_method(self, time: np.ndarray, pulses: np.ndarray, 
                            pulses_per_revolution: int, counting_time_interval: float) -> np.ndarray:
        """
        Pulse Counting Method로 각속도를 추정합니다.
        
        Args:
            time (np.ndarray): 시간 배열
            pulses (np.ndarray): 펄스 배열 (0 또는 1)
            pulses_per_revolution (int): 회전당 펄스 수
            counting_time_interval (float): 펄스 카운팅 시간 간격
            
        Returns:
            np.ndarray: 추정된 각속도 (rad/s)
        """
        estimated_angular_velocity = np.zeros(len(time))
        
        tmp_pulse_count = 0
        prev_time = 0.0
        prev_pulse = 0
        curr_estimated_angular_velocity = 0
        
        for idx in range(len(time)):
            if idx == 0:
                # 초기화
                prev_pulse = pulses[idx]
                prev_time = time[idx]
                continue
            
            # 펄스 상승 엣지 감지 (0 -> 1)
            if xxxxxx == 0 and xxxxxx > 0: # TODO: 펄스 상승 엣지 감지 조건 완성
                tmp_pulse_count += 1
            
            delta_time = time[idx] - prev_time
            
            # 지정된 시간 간격이 지나면 속도 계산
            if delta_time >= xxxxxx: # TODO: 카운팅 시간 간격 조건 완성
                curr_estimated_angular_velocity = xxxxxx # TODO: 각속도 계산 식 완성
                tmp_pulse_count = 0
                prev_time = xxxxxx # TODO: 이전 시간 업데이트
            
            estimated_angular_velocity[idx] = curr_estimated_angular_velocity
            prev_pulse = pulses[idx]
        
        return estimated_angular_velocity
    
    def pulse_timing_method(self, time: np.ndarray, pulses: np.ndarray, 
                          pulses_per_revolution: int) -> np.ndarray:
        """
        Pulse Timing Method로 각속도를 추정합니다.
        
        Args:
            time (np.ndarray): 시간 배열
            pulses (np.ndarray): 펄스 배열 (0 또는 1)
            pulses_per_revolution (int): 회전당 펄스 수
            
        Returns:
            np.ndarray: 추정된 각속도 (rad/s)
        """
        estimated_angular_velocity = np.zeros(len(time))
        
        prev_time = 0.0
        prev_pulse = 0
        curr_estimated_angular_velocity = 0
        
        for idx in range(len(time)):
            if idx == 0:
                # 초기화
                prev_pulse = pulses[idx]
                prev_time = time[idx]
                continue
            
            # 펄스 상승 엣지 감지 (0 -> 1)
            if xxxxxx == 0 and xxxxxx > 0: # TODO: 펄스 상승 엣지 감지 조건 완성
                delta_time = time[idx] - prev_time
                
                # 최소 시간 제한 (너무 짧은 시간 방지)
                if delta_time < 0.001:
                    delta_time = 0.001
                
                curr_estimated_angular_velocity = xxxxxx # TODO: 각속도 계산 식 완성
                prev_time = xxxxxx # TODO: 이전 시간 업데이트
            
            estimated_angular_velocity[idx] = curr_estimated_angular_velocity
            prev_pulse = pulses[idx]
        
        return estimated_angular_velocity
    
    def plot_trajectory_with_time(self, latitudes: np.ndarray, longitudes: np.ndarray, 
                                times: np.ndarray, title: str = "Vehicle Trajectory",
                                imu_data: dict = None):
        """
        시간에 따른 차량 궤적을 시각화합니다.
        """
        plt.figure(figsize=(12, 10))
        
        # 상대 시간 계산 (시작점을 0초로)
        relative_time = times - times[0]
        
        scatter = plt.scatter(longitudes, latitudes, c=relative_time, 
                            cmap='jet', s=10, alpha=0.7)
        
        cbar = plt.colorbar(scatter, label='Time (seconds)')
        
        # IMU 데이터가 있을 경우 최대값 지점 표시
        if imu_data is not None:
            # 시간 동기화를 위해 IMU 시간을 GPS 시간과 맞춤
            imu_time = np.array(imu_data['time'])
            imu_relative_time = imu_time - imu_time[0]
            
            # 각속도 z 최대 절대값 찾기
            angular_vel_z = np.array(imu_data['angular_velocity_z'])
            max_angular_idx = np.argmax(np.abs(angular_vel_z))
            max_angular_time = imu_relative_time[max_angular_idx]
            max_angular_value = angular_vel_z[max_angular_idx]
            
            # 가속도 x, y 최대 절대값 찾기
            accel_x = np.array(imu_data['linear_acceleration_x'])
            accel_y = np.array(imu_data['linear_acceleration_y'])
            
            max_accel_x_idx = np.argmax(np.abs(accel_x))
            max_accel_x_time = imu_relative_time[max_accel_x_idx]
            max_accel_x_value = accel_x[max_accel_x_idx]
            
            max_accel_y_idx = np.argmax(np.abs(accel_y))
            max_accel_y_time = imu_relative_time[max_accel_y_idx]
            max_accel_y_value = accel_y[max_accel_y_idx]
            
            # GPS 궤적에서 해당 시간과 가장 가까운 지점 찾기
            def find_closest_gps_point(target_time):
                time_diff = np.abs(relative_time - target_time)
                return np.argmin(time_diff)
            
            # 최대 각속도 지점
            angular_gps_idx = find_closest_gps_point(max_angular_time)
            plt.plot(longitudes[angular_gps_idx], latitudes[angular_gps_idx], 
                    'x', markersize=12, markeredgecolor='cyan', markeredgewidth=2,
                    label=f'Max |Angular Vel Z|')
            plt.text(longitudes[angular_gps_idx], latitudes[angular_gps_idx], 
                    f'  max |ω_z| = {max_angular_value:.3f} rad/s\n  t = {max_angular_time:.1f}s', 
                    fontsize=9, bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcyan", alpha=0.8, edgecolor='cyan'))
            
            # 최대 가속도 X 지점
            accel_x_gps_idx = find_closest_gps_point(max_accel_x_time)
            plt.plot(longitudes[accel_x_gps_idx], latitudes[accel_x_gps_idx], 
                    'x', markersize=12, markeredgecolor='yellow', markeredgewidth=2,
                    label=f'Max |Accel X|')
            plt.text(longitudes[accel_x_gps_idx], latitudes[accel_x_gps_idx], 
                    f'  max |a_x| = {max_accel_x_value:.3f} m/s²\n  t = {max_accel_x_time:.1f}s', 
                    fontsize=9, bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8, edgecolor='yellow'))
            
            # 최대 가속도 Y 지점
            accel_y_gps_idx = find_closest_gps_point(max_accel_y_time)
            plt.plot(longitudes[accel_y_gps_idx], latitudes[accel_y_gps_idx], 
                    'x', markersize=12, markeredgecolor='lightgreen', markeredgewidth=2,
                    label=f'Max |Accel Y|')
            plt.text(longitudes[accel_y_gps_idx], latitudes[accel_y_gps_idx], 
                    f'  max |a_y| = {max_accel_y_value:.3f} m/s²\n  t = {max_accel_y_time:.1f}s', 
                    fontsize=9, bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8, edgecolor='lightgreen'))
        
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title(title)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # 시작점과 끝점 표시
        plt.plot(longitudes[0], latitudes[0], 'bo', markersize=15, markerfacecolor='none', markeredgecolor='blue', markeredgewidth=3, label='Start Point')
        plt.plot(longitudes[-1], latitudes[-1], 'ro', markersize=15, markerfacecolor='none', markeredgecolor='red', markeredgewidth=3, label='End Point')
        
        plt.legend(loc='upper left')
        plt.tight_layout()
        plt.show()
    
    def plot_imu_data(self, imu_data: dict, title: str = "IMU Data"):
        """
        IMU 데이터를 시각화합니다.
        """
        fig, axes = plt.subplots(1, 2, figsize=(15, 5))
        
        # 시간 배열을 numpy 배열로 변환
        time_array = np.array(imu_data['time'])
        time_relative = time_array - time_array[0]  # 상대 시간
        
        # 각속도 z
        imu_angular_velocity_z_deg = np.array(imu_data['angular_velocity_z']) * 180 / np.pi
        axes[0].plot(time_relative, imu_angular_velocity_z_deg, 'b-', linewidth=1.5)
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('Angular Velocity Z (deg/s)')
        axes[0].set_title('Angular Velocity Z')
        axes[0].grid(True, alpha=0.3)
        
        # 선형가속도 x, y
        axes[1].plot(time_relative, imu_data['linear_acceleration_x'], 'r-', 
                    linewidth=1.5, label='Acceleration X')
        axes[1].plot(time_relative, imu_data['linear_acceleration_y'], 'g-', 
                    linewidth=1.5, label='Acceleration Y')
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Linear Acceleration (m/s²)')
        axes[1].set_title('Linear Acceleration X & Y')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        plt.suptitle(title)
        plt.tight_layout()
        plt.show()
    
    def plot_filtering_comparison(self, time: np.ndarray, raw_data: np.ndarray,
                                avg_filtered: np.ndarray, ma_filtered: np.ndarray,
                                ema_filtered: np.ndarray, title: str = "Filtering Comparison"):
        """
        필터링 결과를 비교 시각화합니다.
        """
        plt.figure(figsize=(12, 8))
        
        plt.plot(time, raw_data, 'k-', alpha=0.7, linewidth=1, label='Raw Data')
        plt.plot(time, avg_filtered, 'r-', linewidth=2, label='Average Filter')
        plt.plot(time, ma_filtered, 'g-', color='lime', linewidth=2, label='Moving Average Filter')
        plt.plot(time, ema_filtered, 'b-', linewidth=2, label='Exponential Moving Average Filter')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Acceleration Y (m/s²)')
        plt.title(title)
        plt.legend(loc='upper right')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    
    def plot_wheel_speed_comparison(self, time: np.ndarray, gt_velocity: np.ndarray,
                                  estimated_velocity: np.ndarray, title: str = "Wheel Speed Estimation"):
        """
        실제 속도와 추정 속도를 비교 시각화합니다.
        """
        plt.figure(figsize=(12, 6))
        
        plt.plot(time, gt_velocity, 'b-', linewidth=2, label='Ground Truth Velocity')
        plt.plot(time, estimated_velocity, 'r--', linewidth=2, label='Estimated Velocity')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.title(title)
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    
    def plot_pulse_data(self, time: np.ndarray, pulses: np.ndarray, title: str = "Pulse Data"):
        """
        펄스 데이터를 시각화합니다.
        """
        plt.figure(figsize=(12, 4))
        plt.plot(time, pulses, 'k-', linewidth=1, label='Pulse Signal')
        plt.xlabel('Time (s)')
        plt.ylabel('Pulse')
        plt.title(title)
        plt.ylim(-0.1, 1.1)
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.show()
    
    def plot_encoder_speed_comparison(self, time: np.ndarray, gt_velocity: np.ndarray,
                                    estimated_m_method: np.ndarray, estimated_t_method: np.ndarray,
                                    title: str = "Encoder Speed Estimation Comparison"):
        """
        실제 속도와 두 가지 추정 방법을 비교 시각화합니다.
        """
        plt.figure(figsize=(14, 8))
        
        plt.plot(time, gt_velocity, 'b-', linewidth=2, label='Ground Truth Velocity')
        plt.plot(time, estimated_m_method, 'r--', linewidth=2, label='Pulse Counting Method')
        plt.plot(time, estimated_t_method, 'g:', linewidth=2, label='Pulse Timing Method')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.title(title)
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
