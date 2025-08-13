"""
 * @file        custom_lat_stanley.py
 * @brief       Stanley 횡방향 제어 알고리즘 구현
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)          
 *
 * @date        2025-08-14 Released by AI Lab, Hansung University
 * 
"""

import numpy as np
from typing import Tuple, Dict, Any

class StanleyController:
    """Stanley 횡방향 제어기"""
    
    def __init__(self, wheelbase: float = 2.7, k_e: float = 0.3, k_v: float = 10.0):
        """
        Args:
            wheelbase: 차량 휠베이스 [m]
            k_e: 횡방향 오차 게인
            k_v: 속도 정규화 상수 [m/s]
        """
        self.wheelbase = wheelbase
        self.k_e = k_e  # 크로스 트랙 에러 게인
        self.k_v = k_v  # 속도 정규화 상수
        self.max_steer = np.deg2rad(30.0)  # 최대 조향각 [rad]
        
    def find_closest_point(self, vehicle_x: float, vehicle_y: float,
                          vehicle_yaw: float,
                          path_x: np.ndarray, path_y: np.ndarray) -> Tuple[int, float, float]:
        """경로에서 가장 가까운 점 찾기"""
        xxxxxx # TODO: 거리 계산식 작성
        
        return closest_idx, path_x[closest_idx], path_y[closest_idx]
    
    def calculate_path_yaw(self, path_x: np.ndarray, path_y: np.ndarray, 
                          closest_idx: int) -> float:
        """경로의 요 각도 계산"""
        if closest_idx < len(path_x) - 1:
            # 현재 점과 다음 점으로 경로 방향 계산
            dx = path_x[closest_idx + 1] - path_x[closest_idx]
            dy = path_y[closest_idx + 1] - path_y[closest_idx]
            path_yaw = np.arctan2(dy, dx)
        elif closest_idx > 0:
            # 마지막 점인 경우 이전 점과 현재 점으로 계산
            dx = path_x[closest_idx] - path_x[closest_idx - 1]
            dy = path_y[closest_idx] - path_y[closest_idx - 1]
            path_yaw = np.arctan2(dy, dx)
        else:
            # 단일 점인 경우
            path_yaw = 0.0
            
        return path_yaw
    
    def calculate_cross_track_error(self, vehicle_x: float, vehicle_y: float,
                                  closest_x: float, closest_y: float,
                                  path_yaw: float) -> float:
        """크로스 트랙 에러 계산 (부호 포함)"""
        xxxxxx # TODO: 크로스 트랙 에러 계산식 작성
        
        return cross_track_error
    
    def stanley_control(self, vehicle_x: float, vehicle_y: float, 
                       vehicle_yaw: float, vehicle_velocity: float,
                       path_x: np.ndarray, path_y: np.ndarray) -> Tuple[float, Dict[str, Any]]:
        """Stanley 제어 계산"""
        
        # 가장 가까운 경로점 찾기
        closest_idx, closest_x, closest_y = self.find_closest_point(
            xxxxxx, xxxxxx, xxxxxx, xxxxxx, xxxxxx) # TODO: 가장 가까운 경로점 찾기
        
        # 경로의 요 각도 계산
        path_yaw = self.calculate_path_yaw(path_x, path_y, closest_idx)
        
        # 크로스 트랙 에러 계산
        cross_track_error = self.calculate_cross_track_error(
            vehicle_x, vehicle_y, closest_x, closest_y, path_yaw) # TODO: 크로스 트랙 에러 계산식 작성
        
        # 헤딩 에러 계산
        yaw_error = xxxxxx # TODO: 헤딩 에러 계산식 작성
        
        # Stanley 제어 법칙
        # δ = ψ_e + arctan(k_e * e / (k_v + v))
        velocity_term = xxxxxx # TODO: 속도 정규화 상수 계산식 작성
        cross_track_term = xxxxxx # TODO: 크로스 트랙 에러 계산식 작성
        
        steering_angle = xxxxxx # TODO: 조향각 계산식 작성
        
        # 조향각 제한
        steering_angle = xxxxxx # TODO: 조향각 제한 계산식 작성
        
        # 제어 정보
        control_info = {
            'closest_point': (xxxxxx, xxxxxx), # TODO: 가장 가까운 경로점 좌표
            'closest_index': xxxxxx, # TODO: 가장 가까운 경로점 인덱스
            'cross_track_error': xxxxxx, # TODO: 크로스 트랙 에러
            'yaw_error': xxxxxx, # TODO: 헤딩 에러
            'path_yaw': xxxxxx, # TODO: 경로 요 각도
            'cross_track_term': xxxxxx # TODO: 크로스 트랙 에러 계산식 작성
        }
        
        return steering_angle, control_info
    
    def normalize_angle(self, angle: float) -> float:
        """각도를 -π ~ π 범위로 정규화"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

def test_stanley():
    """Stanley 제어기 테스트"""
    import matplotlib.pyplot as plt
    
    # 제어기 생성
    controller = StanleyController(wheelbase=2.7, k_e=0.3, k_v=10.0)
    
    # 테스트 경로 생성 (직선 + 곡선)
    # 직선 구간
    x1 = np.linspace(0, 50, 50)
    y1 = np.zeros_like(x1)
    
    # 곡선 구간
    theta = np.linspace(0, np.pi/2, 50)
    radius = 30.0
    x2 = 50 + radius * np.sin(theta)
    y2 = radius * (1 - np.cos(theta))
    
    path_x = np.concatenate([x1, x2])
    path_y = np.concatenate([y1, y2])
    
    # 차량 초기 상태 (경로에서 벗어난 상태)
    vehicle_x, vehicle_y = 25.0, 5.0  # 경로 위쪽으로 5m 벗어남
    vehicle_yaw = np.deg2rad(10.0)  # 10도 헤딩 에러
    vehicle_velocity = 15.0
    
    # Stanley 제어 계산
    steering_angle, info = controller.stanley_control(
        vehicle_x, vehicle_y, vehicle_yaw, vehicle_velocity, path_x, path_y)
    
    # 결과 출력
    print("=== Stanley 제어기 테스트 ===")
    print(f"조향각: {np.rad2deg(steering_angle):.2f}°")
    print(f"크로스 트랙 에러: {info['cross_track_error']:.2f} m")
    print(f"헤딩 에러: {np.rad2deg(info['yaw_error']):.2f}°")
    print(f"경로 요 각도: {np.rad2deg(info['path_yaw']):.2f}°")
    print(f"가장 가까운 점: ({info['closest_point'][0]:.2f}, {info['closest_point'][1]:.2f})")
    
    # 시각화
    plt.figure(figsize=(12, 8))
    plt.plot(path_x, path_y, 'b-', linewidth=2, label='Reference Path')
    plt.plot(vehicle_x, vehicle_y, 'ro', markersize=8, label='Vehicle')
    plt.plot(info['closest_point'][0], info['closest_point'][1], 
             'gs', markersize=8, label='Closest Point')
    
    # 차량 방향 표시
    arrow_length = 5.0
    plt.arrow(vehicle_x, vehicle_y, 
             arrow_length * np.cos(vehicle_yaw), 
             arrow_length * np.sin(vehicle_yaw),
             head_width=1, head_length=1, fc='red', ec='red', label='Vehicle Direction')
    
    # 경로 방향 표시
    plt.arrow(info['closest_point'][0], info['closest_point'][1],
             arrow_length * np.cos(info['path_yaw']), 
             arrow_length * np.sin(info['path_yaw']),
             head_width=1, head_length=1, fc='blue', ec='blue', label='Path Direction')
    
    # 크로스 트랙 에러 표시
    plt.plot([vehicle_x, info['closest_point'][0]], 
             [vehicle_y, info['closest_point'][1]], 
             'r--', linewidth=2, alpha=0.7, label='Cross Track Error')
    
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title('Stanley Controller Test')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    
    # 텍스트 정보 추가
    plt.text(0.02, 0.98, f'Cross Track Error: {info["cross_track_error"]:.2f} m\n'
                         f'Yaw Error: {np.rad2deg(info["yaw_error"]):.2f}°\n'
                         f'Steering: {np.rad2deg(steering_angle):.2f}°', 
             transform=plt.gca().transAxes, verticalalignment='top',
             bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
    
    plt.show()

if __name__ == "__main__":
    test_stanley()
