"""
 * @file        custom_lat_pure_pursuit.py
 * @brief       Pure Pursuit 횡방향 제어 알고리즘 구현
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)          
 *
 * @date        2025-08-14 Released by AI Lab, Hanyang University
 * 
"""

import numpy as np
from typing import Tuple, Dict, Any

class PurePursuitController:
    """Pure Pursuit 횡방향 제어기"""
    
    def __init__(self, wheelbase: float = 2.7, 
                 lookahead_base: float = 5.0, 
                 lookahead_gain: float = 0.3):
        """
        Args:
            wheelbase: 차량 휠베이스 [m]
            lookahead_base: 기본 룩어헤드 거리 [m]
            lookahead_gain: 속도 비례 룩어헤드 게인
        """
        self.wheelbase = wheelbase
        self.lookahead_base = lookahead_base
        self.lookahead_gain = lookahead_gain
        self.max_steer = np.deg2rad(30.0)  # 최대 조향각 [rad]
        
    def calculate_lookahead_distance(self, velocity: float) -> float:
        """속도에 따른 룩어헤드 거리 계산"""
        return xxxxxx # TODO: 룩어헤드 거리 계산식 작성
    
    def find_lookahead_point(self, vehicle_x: float, vehicle_y: float, 
                           vehicle_yaw: float, path_x: np.ndarray, 
                           path_y: np.ndarray, lookahead_distance: float) -> Tuple[float, float, int]:
        """룩어헤드 포인트 찾기"""
        # 차량 좌표계로 변환
        cos_yaw = np.cos(vehicle_yaw)
        sin_yaw = np.sin(vehicle_yaw)
        
        dx = path_x - vehicle_x
        dy = path_y - vehicle_y
        
        # 차량 좌표계에서의 경로점 위치
        x_local = xxxxxx # TODO: 차량 좌표계에서의 경로점 위치 계산식 작성
        y_local = xxxxxx # TODO: 차량 좌표계에서의 경로점 위치 계산식 작성
        
        # 거리 계산
        distances = xxxxxx # TODO: 거리 계산식 작성
        
        # 전방 점들 중에서 룩어헤드 거리와 가장 가까운 점 찾기
        forward_mask = xxxxxx # TODO: 전방 점들 중에서 룩어헤드 거리와 가장 가까운 점 찾기
        
        if np.any(forward_mask):
            forward_distances = xxxxxx # TODO: 전방 점들 중에서 룩어헤드 거리와 가장 가까운 점 찾기
            forward_indices = np.where(forward_mask)[0]
            
            # 룩어헤드 거리와 가장 가까운 점 선택
            distance_diff = xxxxxx # TODO: 룩어헤드 거리와 가장 가까운 점 찾기
            best_idx = forward_indices[np.argmin(distance_diff)]
        else:
            # 전방 점이 없으면 가장 가까운 점 선택
            best_idx = xxxxxx # TODO: 가장 가까운 점 찾기
        
        return path_x[best_idx], path_y[best_idx], best_idx
    
    def pure_pursuit_control(self, vehicle_x: float, vehicle_y: float, 
                           vehicle_yaw: float, vehicle_velocity: float,
                           path_x: np.ndarray, path_y: np.ndarray) -> Tuple[float, Dict[str, Any]]:
        """Pure Pursuit 제어 계산"""
        
        # 룩어헤드 거리 계산
        lookahead_distance = self.calculate_lookahead_distance(vehicle_velocity)
        
        # 룩어헤드 포인트 찾기
        target_x, target_y, target_idx = self.find_lookahead_point(
            xxxxxx, xxxxxx, xxxxxx, xxxxxx, xxxxxx, xxxxxx) # TODO: 룩어헤드 포인트 찾기
        
        # 차량 좌표계에서 타겟 위치
        cos_yaw = np.cos(vehicle_yaw)
        sin_yaw = np.sin(vehicle_yaw)
        
        dx = target_x - vehicle_x
        dy = target_y - vehicle_y
        
        # 차량 좌표계로 변환
        x_local = xxxxxx # TODO: 차량 좌표계에서의 타겟 위치 계산식 작성
        y_local = xxxxxx # TODO: 차량 좌표계에서의 타겟 위치 계산식 작성
        
        # Pure Pursuit 조향각 계산
        # δ = arctan(2 * L * y_local / lookahead_distance^2)
        lookahead_distance_actual = xxxxxx # TODO: 룩어헤드 거리 계산식 작성
        lookahead_distance_actual = xxxxxx # TODO: 분모 보호
        
        steering_angle = xxxxxx # TODO: 조향각 계산식 작성
        
        # 조향각 제한
        steering_angle = xxxxxx # TODO: np.clip 사용하여 조향각 제한
        
        # 제어 정보
        control_info = {
            'lookahead_distance': xxxxxx, # TODO: 룩어헤드 거리
            'lookahead_point': (xxxxxx, xxxxxx), # TODO: 룩어헤드 포인트
            'target_index': xxxxxx, # TODO: 룩어헤드 포인트 인덱스
            'lateral_error': xxxxxx, # TODO: 횡방향 오차
            'lookahead_distance_actual': xxxxxx # TODO: 실제 룩어헤드 거리
        }
        
        return steering_angle, control_info

def test_pure_pursuit():
    """Pure Pursuit 제어기 테스트"""
    import matplotlib.pyplot as plt
    
    # 제어기 생성
    controller = PurePursuitController(wheelbase=2.7, lookahead_base=5.0, lookahead_gain=0.3)
    
    # 테스트 경로 생성 (원형)
    theta = np.linspace(0, 2*np.pi, 100)
    radius = 50.0
    path_x = radius * np.cos(theta)
    path_y = radius * np.sin(theta)
    
    # 차량 초기 상태
    vehicle_x, vehicle_y = 30.0, 0.0
    vehicle_yaw = np.pi/4  # 45도
    vehicle_velocity = 10.0
    
    # Pure Pursuit 제어 계산
    steering_angle, info = controller.pure_pursuit_control(
        xxxxxx, xxxxxx, xxxxxx, xxxxxx, xxxxxx, xxxxxx) # TODO: Pure Pursuit 제어 계산 함수 완성
    
    # 결과 출력
    print("=== Pure Pursuit 제어기 테스트 ===")
    print(f"조향각: {np.rad2deg(steering_angle):.2f}°")
    print(f"룩어헤드 거리: {info['lookahead_distance']:.2f} m")
    print(f"타겟 포인트: ({info['lookahead_point'][0]:.2f}, {info['lookahead_point'][1]:.2f})")
    print(f"횡방향 오차: {info['lateral_error']:.2f} m")
    
    # 시각화
    plt.figure(figsize=(10, 8))
    plt.plot(path_x, path_y, 'b-', linewidth=2, label='Reference Path')
    plt.plot(vehicle_x, vehicle_y, 'ro', markersize=8, label='Vehicle')
    plt.plot(info['lookahead_point'][0], info['lookahead_point'][1], 
             'gs', markersize=8, label='Lookahead Point')
    
    # 차량 방향 표시
    arrow_length = 5.0
    plt.arrow(vehicle_x, vehicle_y, 
             arrow_length * np.cos(vehicle_yaw), 
             arrow_length * np.sin(vehicle_yaw),
             head_width=2, head_length=2, fc='red', ec='red')
    
    # 룩어헤드 거리 원 표시
    circle = plt.Circle((vehicle_x, vehicle_y), info['lookahead_distance'], 
                       fill=False, linestyle='--', color='gray', alpha=0.5)
    plt.gca().add_patch(circle)
    
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title('Pure Pursuit Controller Test')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.show()

if __name__ == "__main__":
    test_pure_pursuit()
