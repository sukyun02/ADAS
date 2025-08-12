"""
 * @file        custom_localization.py
 * @brief       This module provides Extended Kalman Filter implementation for vehicle localization.
 *
 * @authors     Jaehwan Lee (idljh5529@gmail.com)      
 *
 * @date        2025-08-13 Released by AI Lab, Hansung University
 * 
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.animation import FuncAnimation, PillowWriter
import pandas as pd
from typing import Tuple, List, Optional, Dict
import math
import folium
from folium import plugins
from scipy.interpolate import interp1d

# ROS bag 읽기 라이브러리 설정
use_rosbag = False
use_rosbags = False

try:
    import rosbag
    use_rosbag = True
    print("rosbag 라이브러리를 사용합니다.")
except ImportError:
    try:
        from pathlib import Path
        from rosbags.highlevel import AnyReader
        use_rosbags = True
        print("rosbags.highlevel.AnyReader를 사용합니다.")
    except ImportError:
        print("rosbag과 rosbags 라이브러리 모두 사용할 수 없습니다. 시뮬레이션 데이터를 사용합니다.")

class CoordinateTransform:
    """WGS84와 ENU 좌표계 간 변환을 위한 클래스"""
    
    def __init__(self):
        # WGS84 타원체 상수
        self.a = 6378137.0  # 장반경 (m)
        self.f = 1.0 / 298.257223563  # 편평률
        self.e2 = 2 * self.f - self.f * self.f  # 이심률의 제곱
        
    def wgs84_to_enu(self, lat: float, lon: float, alt: float, 
                     lat_ref: float, lon_ref: float, alt_ref: float) -> Tuple[float, float, float]:
        """
        WGS84 좌표를 ENU 좌표로 변환
        
        Args:
            lat, lon, alt: 변환할 WGS84 좌표 (deg, deg, m)
            lat_ref, lon_ref, alt_ref: 기준점 WGS84 좌표 (deg, deg, m)
            
        Returns:
            (east, north, up): ENU 좌표 (m, m, m)
        """
        # 도를 라디안으로 변환
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        lat_ref_rad = math.radians(lat_ref)
        lon_ref_rad = math.radians(lon_ref)
        
        # 기준점에서의 곡률 반지름 계산
        sin_lat_ref = math.sin(lat_ref_rad)
        N = self.a / math.sqrt(1 - self.e2 * sin_lat_ref * sin_lat_ref)
        M = self.a * (1 - self.e2) / math.pow(1 - self.e2 * sin_lat_ref * sin_lat_ref, 1.5)
        
        # 위도, 경도, 고도 차이
        dlat = lat_rad - lat_ref_rad
        dlon = lon_rad - lon_ref_rad
        dalt = alt - alt_ref
        
        # ENU 좌표 계산
        east = (N + alt_ref) * math.cos(lat_ref_rad) * dlon
        north = (M + alt_ref) * dlat
        up = dalt
        
        return east, north, up
    
    def enu_to_wgs84(self, east: float, north: float, up: float,
                     lat_ref: float, lon_ref: float, alt_ref: float) -> Tuple[float, float, float]:
        """
        ENU 좌표를 WGS84 좌표로 변환
        
        Args:
            east, north, up: ENU 좌표 (m, m, m)
            lat_ref, lon_ref, alt_ref: 기준점 WGS84 좌표 (deg, deg, m)
            
        Returns:
            (lat, lon, alt): WGS84 좌표 (deg, deg, m)
        """
        # 기준점을 라디안으로 변환
        lat_ref_rad = math.radians(lat_ref)
        lon_ref_rad = math.radians(lon_ref)
        
        # 기준점에서의 곡률 반지름 계산
        sin_lat_ref = math.sin(lat_ref_rad)
        N = self.a / math.sqrt(1 - self.e2 * sin_lat_ref * sin_lat_ref)
        M = self.a * (1 - self.e2) / math.pow(1 - self.e2 * sin_lat_ref * sin_lat_ref, 1.5)
        
        # WGS84 좌표 계산
        dlat = north / (M + alt_ref)
        dlon = east / ((N + alt_ref) * math.cos(lat_ref_rad))
        dalt = up
        
        lat = math.degrees(lat_ref_rad + dlat)
        lon = math.degrees(lon_ref_rad + dlon)
        alt = alt_ref + dalt
        
        return lat, lon, alt


class SensorDataLoader:
    """ROS bag에서 센서 데이터를 로드하는 클래스"""
    
    def __init__(self):
        self.coord_transform = CoordinateTransform()
    
    def interpolate_covariance_matrices(self, source_time: np.ndarray, source_cov: List, target_time: np.ndarray) -> List:
        """
        Covariance 행렬들을 각 요소별로 선형 보간
        
        Args:
            source_time: 원본 GNSS 시간
            source_cov: 원본 covariance 행렬들 (list of numpy arrays)
            target_time: 목표 IMU 시간
            
        Returns:
            보간된 covariance 행렬들
        """
        # pandas Series 체크
        try:
            if source_cov is None or (hasattr(source_cov, 'empty') and source_cov.empty) or len(source_cov) == 0:
                default_cov = np.eye(3) * 100.0
                return [default_cov.copy() for _ in target_time]
        except Exception as e:
            default_cov = np.eye(3) * 100.0
            return [default_cov.copy() for _ in target_time]
        
        # 원본 covariance 행렬들을 numpy array로 변환
        source_cov_matrices = []
        
        for i in range(len(source_cov)):
            try:
                gnss_cov_matrix = source_cov.iloc[i]
                
                # 다양한 형태의 covariance 데이터 처리
                if isinstance(gnss_cov_matrix, list) and len(gnss_cov_matrix) >= 9:
                    # list 형태를 3x3 행렬로 변환
                    cov_3x3 = np.array(gnss_cov_matrix).reshape(3, 3)
                    source_cov_matrices.append(cov_3x3)
                elif isinstance(gnss_cov_matrix, np.ndarray) and gnss_cov_matrix.size >= 9:
                    # numpy array 형태를 3x3 행렬로 변환
                    cov_3x3 = gnss_cov_matrix.reshape(3, 3)
                    source_cov_matrices.append(cov_3x3)
                elif hasattr(gnss_cov_matrix, '__len__') and len(gnss_cov_matrix) >= 9:
                    # 다른 시퀀스 타입을 3x3 행렬로 변환
                    cov_3x3 = np.array(gnss_cov_matrix).reshape(3, 3)
                    source_cov_matrices.append(cov_3x3)
                else:
                    # 기본값 사용
                    source_cov_matrices.append(np.eye(3) * 100.0)
            except Exception as e:
                # 오류 발생 시 기본값 사용
                source_cov_matrices.append(np.eye(3) * 100.0)
        
        # 각 행렬 요소별로 시간에 따른 배열 생성
        cov_00 = [cov[0, 0] for cov in source_cov_matrices]  # x variance
        cov_11 = [cov[1, 1] for cov in source_cov_matrices]  # y variance  
        cov_22 = [cov[2, 2] for cov in source_cov_matrices]  # z variance
        
        # 각 요소별로 선형 보간
        interp_cov_00 = np.interp(target_time, source_time, cov_00)
        interp_cov_11 = np.interp(target_time, source_time, cov_11)
        interp_cov_22 = np.interp(target_time, source_time, cov_22)
        
        # 보간된 covariance 행렬들 생성
        interpolated_covariances = []
        for i in range(len(target_time)):
            # 대각선 요소만 보간하고 비대각선은 0으로 설정
            cov_matrix = np.array([
                [interp_cov_00[i], 0, 0],
                [0, interp_cov_11[i], 0],
                [0, 0, interp_cov_22[i]]
            ])
            interpolated_covariances.append(cov_matrix)
        
        return interpolated_covariances
    
    def load_from_csv(self, csv_dir: str) -> Dict[str, pd.DataFrame]:
        """
        CSV 파일들에서 센서 데이터 로딩
        
        Args:
            csv_dir: CSV 파일들이 저장된 디렉토리 경로
            
        Returns:
            센서 데이터 딕셔너리
        """
        import os
        from pathlib import Path
        
        csv_path = Path(csv_dir)
        
        # 결과 저장용 딕셔너리
        data = {
            'inspvax': pd.DataFrame(),      # /novatel/oem7/inspvax
            'vehicle_motion_sensor': pd.DataFrame(),  # vehicle_motion_sensor
            'gps_gps': pd.DataFrame(),      # /gps/gps
            'gps_fix': pd.DataFrame(),      # /gps/fix
            'gps_imu': pd.DataFrame(),      # /gps/imu
        }
        
        # CSV 파일명과 데이터 키 매핑
        csv_mapping = {
            'novatel_oem7_inspvax.csv': 'inspvax',
            'vehicle_motion_sensor.csv': 'vehicle_motion_sensor',
            'gps_gps.csv': 'gps_gps', 
            'gps_fix.csv': 'gps_fix',
            'gps_imu.csv': 'gps_imu'
        }
        
        print(f"📁 CSV 디렉토리에서 데이터 로딩: {csv_path}")
        
        # 각 CSV 파일 로딩
        for csv_filename, data_key in csv_mapping.items():
            csv_filepath = csv_path / csv_filename
            
            if csv_filepath.exists():
                try:
                    df = pd.read_csv(csv_filepath)
                    data[data_key] = df
                    print(f"✅ {csv_filename}: {len(df)} 행, {len(df.columns)} 컬럼")
                    
                    # 시간 범위 출력
                    if 'timestamp' in df.columns:
                        time_min = df['timestamp'].min()
                        time_max = df['timestamp'].max()
                        duration = time_max - time_min
                        print(f"   시간 범위: {time_min:.1f} - {time_max:.1f} 초 (총 {duration:.1f}초)")
                        
                except Exception as e:
                    print(f"❌ {csv_filename} 로딩 실패: {e}")
                    
            else:
                print(f"⚠️ {csv_filename} 파일을 찾을 수 없음")
        
        # 로딩된 데이터 요약
        loaded_count = sum(1 for df in data.values() if not df.empty)
        print(f"\n📊 총 {loaded_count}/{len(csv_mapping)} 개 CSV 파일 로딩 완료")
        
        return data
    
    def convert_csv_to_localization_format(self, csv_data: Dict[str, pd.DataFrame]) -> Dict[str, pd.DataFrame]:
        """
        CSV 데이터를 localization에서 사용할 수 있는 형태로 변환
        
        Args:
            csv_data: load_from_csv()에서 반환된 데이터
            
        Returns:
            localization 형태의 센서 데이터
        """
        # 결과 저장용 딕셔너리 (기존 형태와 호환)
        localization_data = {
            'gnss': pd.DataFrame(),
            'imu': pd.DataFrame(),
            'gps_imu': pd.DataFrame(),
            'motion_sensor': pd.DataFrame(),
            'inspvax': pd.DataFrame(),  # 추가: 고정밀 GNSS/INS 데이터
            'odom': pd.DataFrame()
        }
        
        try:
            # 1. INSPVAX 데이터 -> 고정밀 GNSS 데이터로 사용
            if not csv_data['inspvax'].empty:
                inspvax_df = csv_data['inspvax'].copy()
                localization_data['inspvax'] = inspvax_df
                
                # INSPVAX에서 GNSS 형태로 변환 (더 정확한 데이터)
                if all(col in inspvax_df.columns for col in ['latitude', 'longitude', 'height', 'timestamp']):
                    gnss_from_inspvax = pd.DataFrame({
                        'timestamp': inspvax_df['timestamp'],
                        'latitude': inspvax_df['latitude'],
                        'longitude': inspvax_df['longitude'], 
                        'altitude': inspvax_df['height'] if 'height' in inspvax_df.columns else inspvax_df.get('altitude', 0),
                        'covariance': [[0]*9] * len(inspvax_df)  # 기본값
                    })
                    
                    # 표준편차가 있으면 covariance 계산
                    if all(col in inspvax_df.columns for col in ['latitude_stdev', 'longitude_stdev']):
                        covariances = []
                        for _, row in inspvax_df.iterrows():
                            lat_var = row['latitude_stdev'] ** 2
                            lon_var = row['longitude_stdev'] ** 2
                            height_var = row.get('height_stdev', 1.0) ** 2
                            cov = [lat_var, 0, 0, 0, lon_var, 0, 0, 0, height_var]
                            covariances.append(cov)
                        gnss_from_inspvax['covariance'] = covariances
                    
                    localization_data['gnss'] = gnss_from_inspvax
                    print(f"✅ INSPVAX -> GNSS 변환: {len(gnss_from_inspvax)} 포인트")
            
            # 2. Vehicle motion sensor 데이터 -> Wheel Speed 데이터
            if not csv_data['vehicle_motion_sensor'].empty:
                vehicle_motion_sensor_df = csv_data['vehicle_motion_sensor'].copy()
                
                # Wheel speed 필드 확인 및 변환
                wheel_speed_cols = ['wheel_speed_fl', 'wheel_speed_fr', 'wheel_speed_rl', 'wheel_speed_rr']
                if all(col in vehicle_motion_sensor_df.columns for col in wheel_speed_cols):
                    # 평균 속도 계산
                    average_speed = xxxxxx # TODO: 평균 속도 계산 - 4개 바퀴의 평균
                    wheel_speed_data = pd.DataFrame({
                        'timestamp': vehicle_motion_sensor_df['timestamp'],
                        'wheel_speed_fl': vehicle_motion_sensor_df['wheel_speed_fl'],
                        'wheel_speed_fr': vehicle_motion_sensor_df['wheel_speed_fr'],
                        'wheel_speed_rl': vehicle_motion_sensor_df['wheel_speed_rl'], 
                        'wheel_speed_rr': vehicle_motion_sensor_df['wheel_speed_rr'],
                        'average_speed': average_speed
                    })
                    
                    # 추가 motion sensor 데이터도 포함
                    if 'yaw_rate' in vehicle_motion_sensor_df.columns:
                        wheel_speed_data['yaw_rate'] = vehicle_motion_sensor_df['yaw_rate']
                    if 'longitudinal_accel' in vehicle_motion_sensor_df.columns:
                        wheel_speed_data['longitudinal_accel'] = vehicle_motion_sensor_df['longitudinal_accel']
                    if 'lateral_accel' in vehicle_motion_sensor_df.columns:
                        wheel_speed_data['lateral_accel'] = vehicle_motion_sensor_df['lateral_accel']
                    
                    localization_data['motion_sensor'] = wheel_speed_data
                    print(f"✅ Vehicle Motion Sensor -> Motion Sensor 변환: {len(wheel_speed_data)} 포인트")

            ### for test
            # 만약 motion sensor의 시간이 1754911416.2044253 이면, 해당 인덱스로부터 약 1000개의 yawrate 데이터를 음수로 변환
            # 1. sensor_data_csv['motion_sensor']['timestamp']와 1754911416.2044253의 거리를 np.linalg.norm 으로 계산
            # 2. sorting을 통해 가장 가까운 인덱스를 찾고 해당 인덱스로부터 약 1000개의 yawrate 데이터를 음수로 변환
            closest_index = np.argmin(np.linalg.norm(np.abs(localization_data['motion_sensor']['timestamp'] - 1754911416.2044253)))
            closest_index = 85890
            # print(sensor_data_csv['motion_sensor']['timestamp'][0])
            # print(closest_index)
            # print(len(sensor_data_csv['motion_sensor']['yaw_rate']))
            # print(sensor_data_csv['motion_sensor']['yaw_rate'][closest_index])
            for i in range(closest_index, closest_index+2000):
                localization_data['motion_sensor']['yaw_rate'][i] = -localization_data['motion_sensor']['yaw_rate'][i]
            print(localization_data['motion_sensor']['yaw_rate'][closest_index])
            ### for test
            
            # 3. GPS 데이터 (추가 보조용)
            if not csv_data['gps_gps'].empty:
                # /gps/gps/status/satellites_visible과 /gps/gps/status/satellites_used 데이터 저장
                # print(csv_data['gps_gps'].columns)
                # print(csv_data['gps_gps'].columns)
                # print(csv_data['gps_gps']['status'].columns)
                satellites_visible = csv_data['gps_gps']['satellites_visible'].values
                satellites_used = csv_data['gps_gps']['satellites_used'].values
                localization_data['gps_gps'] = pd.DataFrame({
                    'timestamp': csv_data['gps_gps']['timestamp'],
                    'satellites_visible': satellites_visible,
                    'satellites_used': satellites_used
                })
                print(f"✅ GPS GPS 데이터: {len(localization_data['gps_gps'])} 포인트")
                
        except Exception as e:
            print(f"❌ CSV 데이터 변환 중 오류: {e}")
            import traceback
            traceback.print_exc()
        
        # 변환 결과 요약
        print(f"\n📋 Localization 데이터 변환 완료:")
        for key, df in localization_data.items():
            if not df.empty:
                print(f"   ✅ {key}: {len(df)} 포인트")
            else:
                print(f"   ⚠️ {key}: 데이터 없음")
        
        return localization_data

class LocalizationVisualizer:
    """위치 추정 결과 시각화 클래스"""
    
    def __init__(self):
        # plt.rcParams['font.family'] = ['DejaVu Sans', 'Arial Unicode MS', 'Malgun Gothic']
        self.coord_transform = CoordinateTransform()
    
    def plot_estimation_results(self, gnss_data: dict, ekf_results: Dict, converted_enu):
        """
        EKF 추정 결과 상세 분석 (새로운 3-DOF state)
        
        Args:
            gnss_data: GNSS 센서 데이터
            ekf_results: EKF 결과 딕셔너리 - state: [x, y, yaw]
        """
        if ekf_results['states'].shape[0] != 3:
            print("⚠️ 3-DOF 상태 벡터가 필요합니다 [x, y, yaw]")
            print(f"현재 shape: {ekf_results['states'].shape}")
            return
            
        time_data = ekf_results['timestamps']
        states = ekf_results['states']  # (3, N) 형태: [x, y, yaw]
        covariances = ekf_results['covariances']  # (3, 3, N) 형태
        
        # 표준편차 계산 (대각선 성분의 제곱근)
        std_data = np.sqrt(np.diagonal(covariances, axis1=0, axis2=1))  # (3, N) 형태
        gnss_enu = converted_enu
        
        # GNSS 데이터 보간
        gnss_time_data = gnss_data['timestamp']
        # gnss_enu_interp = self._interpolate_gnss_data(gnss_enu, gnss_time_data, time_data)
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 16))
        
        # 위치 X 추정 (state: [x, y, v, yaw])
        print(states.shape)
        print(std_data.shape)
        print(time_data.shape)
        print(states[0, :].shape)
        print(std_data[:, 0].shape)
        axes[0].fill_between(time_data,
                           states[0, :] + std_data[:, 0],
                           states[0, :] - std_data[:, 0],
                           alpha=0.6, color='lightblue', label='EKF 1σ Uncertainty')
        # axes[0].plot(time_data, states[0, :], 'b-', linewidth=2, label='EKF X')
        # axes[0].plot(time_data, gnss_enu_interp[:, 0], 'r--', linewidth=2, alpha=0.7, label='GNSS X (Interpolated)')
        axes[0].scatter(gnss_time_data, gnss_enu[:, 0], c='red', s=3, alpha=0.8, label='GNSS Raw')
        axes[0].plot(time_data, states[0, :], 'b-', linewidth=2, label='EKF X')
        axes[0].set_xlabel('Time [s]')
        axes[0].set_ylabel('East Position [m]')
        axes[0].set_title('Estimated Position X')
        axes[0].grid(True)
        axes[0].legend()
        
        # 위치 Y 추정
        axes[1].fill_between(time_data,
                           states[1, :] + std_data[:, 1],
                           states[1, :] - std_data[:, 1],
                           alpha=0.6, color='lightblue', label='EKF 1σ Uncertainty')
        # axes[1].plot(time_data, states[1, :], 'b-', linewidth=2, label='EKF Y')
        # axes[1].plot(time_data, gnss_enu_interp[:, 1], 'r--', linewidth=2, alpha=0.7, label='GNSS Y (Interpolated)')
        axes[1].scatter(gnss_time_data, gnss_enu[:, 1], c='red', s=3, alpha=0.8, label='GNSS Raw')
        axes[1].plot(time_data, states[1, :], 'b-', linewidth=2, label='EKF Y')
        axes[1].set_xlabel('Time [s]')
        axes[1].set_ylabel('North Position [m]')
        axes[1].set_title('Estimated Position Y')
        axes[1].grid(True)
        axes[1].legend()
        
        # 요각 (yaw) 추정 - 새로운 state의 4번째 요소
        axes[2].fill_between(time_data,
                           states[2, :] + std_data[:, 2],
                           states[2, :] - std_data[:, 2],
                           alpha=0.6, color='orange', label='EKF 1σ Uncertainty')
        axes[2].plot(time_data, states[2, :], 'orange', linewidth=2, label='EKF Yaw')
        axes[2].set_xlabel('Time [s]')
        axes[2].set_ylabel('Yaw [rad]')
        axes[2].set_title('Estimated Yaw Angle')
        axes[2].grid(True)
        axes[2].legend()
        
        # GNSS 업데이트 위치 표시 (있는 경우)
        if 'gnss_updates' in ekf_results:
            gnss_update_times = time_data[ekf_results['gnss_updates']]
            for ax in axes:
                for update_time in gnss_update_times[::5]:  # 5개마다 표시
                    ax.axvline(x=update_time, color='red', alpha=0.3, linestyle=':', linewidth=1)
        
        plt.tight_layout()
        plt.show()
    
    def plot_covariance_evolution(self, gnss_data: dict, ekf_results: Dict):
        """
        공분산 진화 분석
        
        Args:
            gnss_data: GNSS 센서 데이터
            ekf_results: EKF 결과 딕셔너리
        """
        time_data = ekf_results['timestamps']
        covariances = ekf_results['covariances']
        
        plt.figure(figsize=(12, 8))
        
        # 위치 공분산 (X, Y)
        plt.semilogy(time_data, covariances[0, 0, :], 'r-', linewidth=2, 
                    label='Position X Covariance')
        plt.semilogy(time_data, covariances[1, 1, :], 'g-', linewidth=2,
                    label='Position Y Covariance')

        # 요각 공분산
        plt.semilogy(time_data, covariances[2, 2, :], 'm-', linewidth=2,
                    label='Yaw Covariance')
        
        plt.xlabel('Time [s]')
        plt.ylabel('Covariance')
        plt.title('EKF Covariance Evolution Over Time')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.show()
    
    def plot_covariance_ellipses(self, gnss_data: dict, ekf_results: Dict, sigma: float = 1.0,
                               time_points: List[int] = None):
        """
        특정 시점에서의 공분산 타원 시각화
        
        Args:
            gnss_data: GNSS 센서 데이터
            ekf_results: EKF 결과 딕셔너리
            time_points: 표시할 시점들의 인덱스 리스트
            sigma: 시그마 배수
        """
        time_data = ekf_results['timestamps']
        states = ekf_results['states']
        covariances = ekf_results['covariances']
        ref_point = ekf_results['ref_point']
        
        # GNSS 데이터를 ENU로 변환
        gnss_enu = []
        for i in range(len(gnss_data)):
            east, north, up = self.coord_transform.wgs84_to_enu(
                gnss_data['latitude'][i], gnss_data['longitude'][i], gnss_data['altitude'][i],
                ref_point[0], ref_point[1], ref_point[2]
            )
            gnss_enu.append([east, north])
        gnss_enu = np.array(gnss_enu)
        
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        
        # 전체 궤적
        ax.plot(states[0, :], states[1, :], 'b-', linewidth=2, label='EKF Trajectory', alpha=0.7)
        ax.plot(gnss_enu[:, 0], gnss_enu[:, 1], 'ro-', markersize=3, linewidth=1, 
               label='GNSS Measurements', alpha=0.6)
        
        # 시작점과 끝점 표시
        ax.plot(states[0, 0], states[1, 0], 'go', markersize=10, label='Start')
        ax.plot(states[0, -1], states[1, -1], 'ro', markersize=10, label='End')
        
        # 시점 설정
        if time_points is None:
            n_points = min(5, len(time_data))
            time_points = np.linspace(0, len(time_data)-1, n_points, dtype=int)
        
        colors = ['red', 'orange', 'green', 'blue', 'purple']
        
        for i, (time_idx, color) in enumerate(zip(time_points, colors)):
            if time_idx < len(time_data):
                center = states[:2, time_idx]
                cov_matrix = covariances[:2, :2, time_idx]
                
                ellipse = self._plot_covariance_ellipse(
                    center, cov_matrix, n_sigma=sigma,
                    facecolor=color, edgecolor=color, alpha=0.3
                )
                ax.add_patch(ellipse)
                
                # 시간 라벨
                ax.text(center[0], center[1], f't={time_data[time_idx]:.1f}s', 
                       fontsize=10, ha='center', va='center',
                       bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        ax.set_xlabel('East Position [m]')
        ax.set_ylabel('North Position [m]')
        ax.set_title(f'EKF Trajectory with Covariance Ellipses ({sigma}σ)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')
        
        plt.tight_layout()
        plt.show()
        
        print("🔍 공분산 타원 해석:")
        print("- 타원의 크기: 위치 불확실성의 크기")
        print("- 타원의 방향: 불확실성의 주축 방향")
        print("- GNSS 측정 업데이트 후: 타원이 작아짐 (불확실성 감소)")
        print("- 예측 단계: 타원이 커짐 (불확실성 증가)")
    
    def _plot_covariance_ellipse_kf(self, center: np.ndarray, cov_matrix: np.ndarray, 
                               n_sigma: float = 1.0, **kwargs) -> Ellipse:
        """
        공분산 타원 생성 (7번 칼만 필터와 동일)
        
        Args:
            center: 중심점 [x, y]
            cov_matrix: 2x2 공분산 행렬
            n_sigma: 시그마 배수
            
        Returns:
            Ellipse 객체
        """
        # 고유값과 고유벡터 계산
        eigenvals, eigenvecs = np.linalg.eigh(cov_matrix)
        
        # 타원의 장축, 단축 길이 계산 (카이제곱 분포 기반)
        chi2_val = 2.2789 if n_sigma == 1.0 else n_sigma**2 * 2  # 2 DOF, 1-sigma
        width = 2 * np.sqrt(chi2_val * eigenvals[0])
        height = 2 * np.sqrt(chi2_val * eigenvals[1])
        
        # 회전각 계산
        angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
        
        # 타원 생성
        ellipse = Ellipse(center, width, height, angle=angle, 
                         fill=kwargs.get('fill', True), 
                         facecolor=kwargs.get('facecolor', 'yellow'),
                         edgecolor=kwargs.get('edgecolor', 'red'),
                         alpha=kwargs.get('alpha', 0.5))
        
        return ellipse
    
    def create_localization_animation(self, gnss_data: dict,
                                    converted_enu: np.ndarray,
                                    ekf_results: Dict,
                                    ref_point: List[float],
                                    save_path: str = None,
                                    fps: int = 10,
                                    figsize: tuple = (12, 10),
                                    data_fraction: float = 1.0,
                                    data_region: str = 'all',
                                    sampling_step: int = 1) -> None:
        """
        EKF 위치 추정 애니메이션 생성 (속도 최적화 옵션 포함)
        
        Args:
            gnss_data: GNSS 센서 데이터
            ekf_results: EKF 결과 딕셔너리  
            ref_point: 기준점 [lat, lon, alt]
            save_path: 저장 경로 (None이면 화면 표시)
            fps: 프레임 레이트
            figsize: 그림 크기
            data_fraction: 사용할 데이터 비율 (0.0-1.0, 기본값: 1.0 = 전체)
            data_region: 데이터 영역 선택 ('all', 'start', 'end', 기본값: 'all')
            sampling_step: 샘플링 간격 (1=모든 프레임, 5=5프레임마다 1개, 기본값: 1)
        """
        gnss_enu = converted_enu
        
        # 데이터 준비 (EKF 결과 구조 수정)
        ekf_positions_full = ekf_results['states'][:2, :]  # (2, N) 형태
        ekf_time_data_full = ekf_results['timestamps']
        gnss_time_data_full = gnss_data['timestamp']
        covariances_full = ekf_results['covariances']  # (4, 4, N) 형태 (4-DOF)
        
        # pandas Series나 다른 타입을 numpy array로 변환하여 인덱싱 문제 해결
        if hasattr(ekf_time_data_full, 'values'):
            ekf_time_data_full = ekf_time_data_full.values
        elif isinstance(ekf_time_data_full, list):
            ekf_time_data_full = np.array(ekf_time_data_full)
            
        if hasattr(gnss_time_data_full, 'values'):
            gnss_time_data_full = gnss_time_data_full.values
        elif isinstance(gnss_time_data_full, list):
            gnss_time_data_full = np.array(gnss_time_data_full)
            
        n_frames_full = len(ekf_time_data_full)
        
        # 데이터 샘플링/구간 선택 적용
        if data_fraction < 1.0 or data_region != 'all' or sampling_step > 1:
            print(f"🚀 성능 최적화 적용:")
            print(f"   - 데이터 비율: {data_fraction*100:.0f}%")
            print(f"   - 구간 선택: {data_region}")
            print(f"   - 샘플링 간격: {sampling_step}")
            
            # 구간 선택
            if data_region == 'start':
                end_idx = int(n_frames_full * data_fraction)
                start_idx = 0
            elif data_region == 'end':
                start_idx = int(n_frames_full * (1.0 - data_fraction))
                end_idx = n_frames_full
            else:  # 'all'
                total_samples = int(n_frames_full * data_fraction)
                start_idx = (n_frames_full - total_samples) // 2
                end_idx = start_idx + total_samples
            
            # 샘플링 적용
            frame_indices = np.arange(start_idx, end_idx, sampling_step)
            
            # 선택된 인덱스로 데이터 추출
            ekf_positions = ekf_positions_full[:, frame_indices]
            ekf_time_data = ekf_time_data_full[frame_indices]
            covariances = covariances_full[:, :, frame_indices]
            
            print(f"   - 원본 프레임: {n_frames_full} → 최적화 프레임: {len(frame_indices)}")
            print(f"   - 예상 생성 시간: {len(frame_indices)/60:.1f}분 (원본 대비 {len(frame_indices)/n_frames_full*100:.1f}%)")
        else:
            ekf_positions = ekf_positions_full
            ekf_time_data = ekf_time_data_full
            covariances = covariances_full
            frame_indices = np.arange(n_frames_full)
        
        n_frames = len(frame_indices)
        
        # ekf_time_data가 pandas Series인 경우 numpy array로 변환
        if hasattr(ekf_time_data, 'values'):
            ekf_time_data = ekf_time_data.values
        elif isinstance(ekf_time_data, list):
            ekf_time_data = np.array(ekf_time_data)
        
        # # GNSS 데이터를 EKF 시간에 맞춰 보간
        # gnss_enu_interp = self._interpolate_gnss_data(gnss_enu, gnss_time_data_full, ekf_time_data)
        
        # 그림 초기화
        fig, ax = plt.subplots(figsize=figsize)
        
        # 전체 궤적 미리 그리기 (연한 색)
        ax.plot(gnss_enu[:, 0], gnss_enu[:, 1], 'r-', 
               linewidth=1, alpha=0.3, label='GNSS Path (Interpolated)')
        ax.plot(ekf_positions[0, :], ekf_positions[1, :], 'b--', 
               linewidth=1, alpha=0.3, label='EKF Path')
        
        if 'ground_truth' in ekf_results:
            gt_pos = ekf_results['ground_truth']
            ax.plot(gt_pos[:, 0], gt_pos[:, 1], 'g-', 
                   linewidth=1, alpha=0.3, label='GT Path')
        
        # 애니메이션 요소 초기화
        current_gnss, = ax.plot([], [], 'ro', markersize=8, 
                               markerfacecolor='red', markeredgecolor='white',
                               markeredgewidth=2, label='Current GNSS')
        current_ekf, = ax.plot([], [], 'bo', markersize=8,
                              markerfacecolor='blue', markeredgecolor='white',
                              markeredgewidth=2, label='Current EKF')
        
        # 궤적 추적선
        trail_gnss, = ax.plot([], [], 'r-', linewidth=2, alpha=0.8)
        trail_ekf, = ax.plot([], [], 'b-', linewidth=2, alpha=0.8)
        
        if 'ground_truth' in ekf_results:
            current_gt, = ax.plot([], [], 'go', markersize=8,
                                 markerfacecolor='green', markeredgecolor='white',
                                 markeredgewidth=2, label='Current GT')
            trail_gt, = ax.plot([], [], 'g-', linewidth=2, alpha=0.8)
        
        # 불확실성 타원
        ellipse_patch = None
        
        # 시간 텍스트
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                           fontsize=14, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        # 축 설정
        margin = 20
        x_min = min(np.min(gnss_enu[:, 0]), np.min(ekf_positions[0, :])) - margin
        x_max = max(np.max(gnss_enu[:, 0]), np.max(ekf_positions[0, :])) + margin
        y_min = min(np.min(gnss_enu[:, 1]), np.min(ekf_positions[1, :])) - margin
        y_max = max(np.max(gnss_enu[:, 1]), np.max(ekf_positions[1, :])) + margin
        
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel('East [m]')
        ax.set_ylabel('North [m]')
        ax.set_title('EKF Localization Animation')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        ax.set_aspect('equal')
        
        def animate(frame):
            nonlocal ellipse_patch
            
            # 현재 시간 (numpy array 접근 사용)
            current_time = ekf_time_data[frame]
            
            # 현재 위치 업데이트 
            gnss_pos = gnss_enu[frame]
            ekf_pos = ekf_positions[:, frame]
            
            current_gnss.set_data([gnss_pos[0]], [gnss_pos[1]])
            current_ekf.set_data([ekf_pos[0]], [ekf_pos[1]])
            
            # 궤적 추적선 업데이트
            trail_gnss.set_data(gnss_enu[:frame+1, 0], gnss_enu[:frame+1, 1])
            trail_ekf.set_data(ekf_positions[0, :frame+1], ekf_positions[1, :frame+1])
            
            if 'ground_truth' in ekf_results:
                gt_pos_current = gt_pos[frame]
                current_gt.set_data([gt_pos_current[0]], [gt_pos_current[1]])
                trail_gt.set_data(gt_pos[:frame+1, 0], gt_pos[:frame+1, 1])
            
            # 불확실성 타원 업데이트
            if ellipse_patch is not None:
                ellipse_patch.remove()
            
            cov_matrix = covariances[:2, :2, frame]  # 위치 공분산만
            ellipse_patch = self._plot_covariance_ellipse(
                ekf_pos, cov_matrix, n_sigma=1.0,
                facecolor='cyan', edgecolor='blue', alpha=0.4
            )
            ax.add_patch(ellipse_patch)

            x_min = min(gnss_pos[0], ekf_pos[0]) - margin
            x_max = max(gnss_pos[0], ekf_pos[0]) + margin
            y_min = min(gnss_pos[1], ekf_pos[1]) - margin
            y_max = max(gnss_pos[1], ekf_pos[1]) + margin
            
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
            
            # 시간 텍스트 업데이트
            time_text.set_text(f'Time: {current_time:.2f}s\nFrame: {frame+1}/{n_frames}')
            
            elements = [current_gnss, current_ekf, trail_gnss, trail_ekf, 
                       ellipse_patch, time_text]
            
            if 'ground_truth' in ekf_results:
                elements.extend([current_gt, trail_gt])
            
            # 진행률 터미널에 표시
            if frame % 20 == 0:
                progress = (frame+1) / n_frames
                print(f'Progress: {progress*100:.1f}%', end='\r')
            
            return elements
        
        # 애니메이션 생성
        print(f"애니메이션 생성 중... (총 {n_frames}프레임, {fps}fps)")
        anim = FuncAnimation(fig, animate, frames=n_frames, interval=1000//fps,
                           blit=False, repeat=True)
        
        # 저장 또는 표시
        if save_path:
            print(f"애니메이션을 {save_path}에 저장 중...")
            if save_path.endswith('.gif'):
                writer = PillowWriter(fps=fps)
                anim.save(save_path, writer=writer)
            elif save_path.endswith('.mp4'):
                anim.save(save_path, writer='ffmpeg', fps=fps)
            print("✅ 애니메이션 저장 완료!")
        else:
            plt.show()
        
        return anim
    
    def _plot_covariance_ellipse(self, center: np.ndarray, cov_matrix: np.ndarray,
                                n_sigma: float = 1.0, **kwargs) -> Ellipse:
        """공분산 타원 생성"""
        eigenvals, eigenvecs = np.linalg.eigh(cov_matrix)
        
        chi2_val = 2.2789 if n_sigma == 1.0 else n_sigma**2 * 2
        width = 2 * np.sqrt(chi2_val * eigenvals[0])
        height = 2 * np.sqrt(chi2_val * eigenvals[1])
        
        angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
        
        ellipse = Ellipse(center, width, height, angle=angle,
                         fill=kwargs.get('fill', True),
                         facecolor=kwargs.get('facecolor', 'yellow'),
                         edgecolor=kwargs.get('edgecolor', 'red'),
                         alpha=kwargs.get('alpha', 0.5))
        
        return ellipse
    
    def _interpolate_gnss_data(self, gnss_enu: np.ndarray, gnss_time: np.ndarray, 
                              target_time: np.ndarray) -> np.ndarray:
        """
        GNSS ENU 데이터를 목표 시간에 맞춰 선형 보간
        
        Args:
            gnss_enu: GNSS ENU 위치 데이터 [N x 2]
            gnss_time: GNSS 시간 데이터 [N]
            target_time: 목표 시간 데이터 [M]
            
        Returns:
            보간된 GNSS ENU 데이터 [M x 2]
        """
        if len(gnss_time) < 2:
            # 데이터가 부족한 경우 첫 번째 값으로 채움
            return np.tile(gnss_enu[0], (len(target_time), 1))
        
        # 시간 범위 확인
        time_min = max(gnss_time.min(), target_time.min())
        time_max = min(gnss_time.max(), target_time.max())
        
        # 보간 가능한 범위로 제한
        valid_mask = (target_time >= time_min) & (target_time <= time_max)
        
        # 결과 배열 초기화
        interpolated = np.zeros((len(target_time), 2))
        
        if np.any(valid_mask):
            # X, Y 좌표 각각 보간
            interp_x = interp1d(gnss_time, gnss_enu[:, 0], kind='linear', 
                               bounds_error=False, fill_value='extrapolate')
            interp_y = interp1d(gnss_time, gnss_enu[:, 1], kind='linear',
                               bounds_error=False, fill_value='extrapolate')
            
            interpolated[:, 0] = interp_x(target_time)
            interpolated[:, 1] = interp_y(target_time)
        else:
            # 보간 불가능한 경우 첫 번째 값으로 채움
            interpolated[:] = gnss_enu[0]
        
        return interpolated
    
    def create_satellite_map_visualization(self, gnss_data: pd.DataFrame,
                                         ekf_results: Dict,
                                         ref_point: List[float],
                                         save_path: str = "localization_map.html"):
        """
        위성 지도 위에 결과 시각화
        """
        # EKF 결과를 WGS84로 변환
        ekf_positions_enu = ekf_results['states'][:2, :]
        ekf_wgs84 = []
        
        for pos in ekf_positions_enu.T:
            lat, lon, alt = self.coord_transform.enu_to_wgs84(
                pos[0], pos[1], 0, ref_point[0], ref_point[1], ref_point[2]
            )
            ekf_wgs84.append([lat, lon])
        
        ekf_wgs84 = np.array(ekf_wgs84)
        
        # 지도 중심점 계산
        center_lat = np.mean(gnss_data['latitude'])
        center_lon = np.mean(gnss_data['longitude'])
        
        # Folium 지도 생성
        m = folium.Map(
            location=[center_lat, center_lon],
            zoom_start=16,
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri WorldImagery'
        )
        
        # GNSS 원시 데이터 (빨간색)
        gnss_coords = list(zip(gnss_data['latitude'], gnss_data['longitude']))
        folium.PolyLine(
            gnss_coords,
            color='red',
            weight=3,
            opacity=0.8,
            popup='GNSS Raw Data'
        ).add_to(m)
        
        # EKF 추정 결과 (파란색)
        ekf_coords = list(zip(ekf_wgs84[:, 0], ekf_wgs84[:, 1]))
        folium.PolyLine(
            ekf_coords,
            color='blue',
            weight=3,
            opacity=0.8,
            popup='EKF Estimation'
        ).add_to(m)
        
        # Ground Truth가 있다면 추가 (녹색)
        if 'ground_truth' in ekf_results:
            gt_positions_enu = ekf_results['ground_truth']
            gt_wgs84 = []
            for pos in gt_positions_enu:
                lat, lon, alt = self.coord_transform.enu_to_wgs84(
                    pos[0], pos[1], 0, ref_point[0], ref_point[1], ref_point[2]
                )
                gt_wgs84.append([lat, lon])
            
            gt_coords = list(zip(np.array(gt_wgs84)[:, 0], np.array(gt_wgs84)[:, 1]))
            folium.PolyLine(
                gt_coords,
                color='green',
                weight=3,
                opacity=0.8,
                popup='Ground Truth'
            ).add_to(m)
        
        # 시작점과 끝점 마커
        folium.Marker(
            [gnss_data.iloc[0]['latitude'], gnss_data.iloc[0]['longitude']],
            popup='Start Point',
            icon=folium.Icon(color='green', icon='play')
        ).add_to(m)
        
        folium.Marker(
            [gnss_data.iloc[-1]['latitude'], gnss_data.iloc[-1]['longitude']],
            popup='End Point',
            icon=folium.Icon(color='red', icon='stop')
        ).add_to(m)
        
        # 범례 추가
        legend_html = '''
        <div style="position: fixed; 
                    bottom: 50px; left: 50px; width: 200px; height: 90px; 
                    background-color: white; border:2px solid grey; z-index:9999; 
                    font-size:14px; padding: 10px">
        <b>Legend</b><br>
        <i class="fa fa-minus" style="color:red"></i> GNSS Raw Data<br>
        <i class="fa fa-minus" style="color:blue"></i> EKF Estimation<br>
        <i class="fa fa-minus" style="color:green"></i> Ground Truth
        </div>
        '''
        m.get_root().html.add_child(folium.Element(legend_html))
        
        # 지도 저장
        m.save(save_path)
        print(f"위성 지도가 {save_path}에 저장되었습니다.")
        
        return m