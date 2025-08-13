"""
 * @file        custom_long_pid.py
 * @brief       PID 종방향 제어 알고리즘 구현
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)          
 *
 * @date        2025-08-14 Released by AI Lab, Hansung University
 * 
"""

import numpy as np
from typing import Tuple, Dict, Any

class PIDController:
    """PID 종방향 제어기"""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.1, kd: float = 0.05, 
                 dt: float = 0.02, max_output: float = 1.0, min_output: float = -1.0):
        """
        Args:
            kp: 비례 게인
            ki: 적분 게인  
            kd: 미분 게인
            dt: 샘플링 시간 [s]
            max_output: 최대 출력 (가속 페달)
            min_output: 최소 출력 (브레이크 페달)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_output = max_output
        self.min_output = min_output
        
        # PID 내부 변수
        self.integral = 0.0
        self.previous_error = 0.0
        self.is_initialized = False
        
        # 적분 윈드업 방지
        self.integral_max = 10.0
        self.integral_min = -10.0
        
    def reset(self):
        """PID 제어기 초기화"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.is_initialized = False
        
    def update(self, target_velocity: float, current_velocity: float) -> Tuple[float, Dict[str, Any]]:
        """PID 제어 업데이트"""
        
        # 오차 계산
        error = xxxxxx # TODO: 오차 계산식 작성
        
        # 초기화 체크
        if not self.is_initialized:
            self.previous_error = xxxxxx # TODO: 이전 오차 저장
            self.is_initialized = True
        
        # 비례항 (P)
        proportional = xxxxxx # TODO: 비례항 계산식 작성
        
        # 적분항 (I)
        self.integral += xxxxxx # TODO: 적분항 계산식 작성
        # 적분 윈드업 방지
        self.integral = xxxxxx # TODO: 적분 윈드업 방지
        integral_term = xxxxxx # TODO: 적분항 계산식 작성
        
        # 미분항 (D)
        derivative = xxxxxx # TODO: 미분항 계산식 작성
        derivative_term = xxxxxx # TODO: 미분항 계산식 작성
        
        # PID 출력
        output = xxxxxx # TODO: PID 출력 계산식 작성
        
        # 출력 제한
        output = xxxxxx # TODO: 출력 제한 계산식 작성
        
        # 다음 계산을 위해 현재 오차 저장
        self.previous_error = error
        
        # 제어 정보
        control_info = {
            'error': xxxxxx, # TODO: 오차
            'proportional': xxxxxx, # TODO: 비례항
            'integral': xxxxxx, # TODO: 적분항
            'derivative': xxxxxx, # TODO: 미분항
            'integral_sum': xxxxxx, # TODO: 적분항 합
            'output': output
        }
        
        return output, control_info

class AdaptiveSpeedController:
    """적응형 속도 제어기 (곡률 기반 속도 조절)"""
    
    def __init__(self, base_speed: float = 15.0, curvature_factor: float = 10.0):
        """
        Args:
            base_speed: 기본 목표 속도 [m/s]
            curvature_factor: 곡률 기반 속도 감소 인수
        """
        self.base_speed = base_speed
        self.curvature_factor = curvature_factor
        self.min_speed = 3.0  # 최소 속도 [m/s]
        self.max_speed = 30.0  # 최대 속도 [m/s]
        
    def calculate_curvature(self, path_x: np.ndarray, path_y: np.ndarray, 
                          current_idx: int, lookahead: int = 5) -> float:
        """경로의 곡률 계산"""
        if len(path_x) < 3 or current_idx >= len(path_x) - lookahead:
            return 0.0
        
        # 3점을 이용한 곡률 계산
        idx1 = xxxxxx # TODO: 첫 번째 점 인덱스
        idx2 = current_idx
        idx3 = xxxxxx # TODO: 현재 점 + lookahead 번째 점 인덱스
        
        x1, y1 = xxxxxx # TODO: 첫 번째 점 좌표
        x2, y2 = xxxxxx # TODO: 두 번째 점 좌표
        x3, y3 = xxxxxx # TODO: 세 번째 점 좌표
        
        # 곡률 계산 공식
        # K = 2 * |det| / (a * b * c)
        # 여기서 det = (x2-x1)(y3-y1) - (y2-y1)(x3-x1)
        det = xxxxxx # TODO: 곡률 계산식 작성
        
        a = xxxxxx # TODO: 곡률 계산식 작성
        b = xxxxxx # TODO: 곡률 계산식 작성
        c = xxxxxx # TODO: 곡률 계산식 작성
        
        if xxxxxx < 1e-6:
            return 0.0
        
        curvature = xxxxxx # TODO: 곡률 계산식 작성
        return curvature
    
    def get_target_speed(self, path_x: np.ndarray, path_y: np.ndarray, 
                        current_idx: int) -> float:
        """곡률 기반 목표 속도 계산"""
        curvature = self.calculate_curvature(path_x, path_y, current_idx) # TODO: 곡률 계산식 작성
        
        # 곡률이 클수록 속도 감소
        speed_reduction = xxxxxx # TODO: 곡률 기반 속도 감소 인수 계산식 작성 (self.curvature_factor 사용)
        target_speed = xxxxxx # TODO: 목표 속도 계산식 작성
        
        # 속도 제한
        target_speed = xxxxxx # TODO: 속도 제한 계산식 작성
        
        return target_speed

def test_pid_controller():
    """PID 제어기 테스트"""
    import matplotlib.pyplot as plt
    
    # PID 제어기 생성
    pid = PIDController(kp=1.5, ki=0.1, kd=0.05, dt=0.02)
    
    # 시뮬레이션 파라미터
    dt = 0.02
    duration = 10.0
    time = np.arange(0, duration, dt)
    
    # 목표 속도 시나리오
    target_velocities = []
    for t in time:
        if t < 3.0:
            target_vel = 10.0  # 10 m/s로 가속
        elif t < 6.0:
            target_vel = 3.0  # 20 m/s로 증속
        else:
            target_vel = 5.0   # 5 m/s로 감속
        target_velocities.append(target_vel)
    
    # 시뮬레이션 실행
    current_velocity = 0.0
    velocities = []
    control_outputs = []
    errors = []
    
    for i, target_vel in enumerate(target_velocities):
        # PID 제어 계산
        control_output, info = pid.update(target_vel, current_velocity)
        
        # 간단한 차량 모델 (1차 시스템)
        # τ * dv/dt + v = K * u
        tau = 1.0  # 시간 상수
        K = 10.0   # 정적 게인
        
        # 속도 업데이트
        velocity_dot = (-current_velocity + K * control_output) / tau
        current_velocity += velocity_dot * dt
        current_velocity = max(0.0, current_velocity)  # 음수 속도 방지
        
        # 데이터 저장
        velocities.append(current_velocity)
        control_outputs.append(control_output)
        errors.append(info['error'])
    
    # 결과 시각화
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # 속도 추종 성능
    axes[0].plot(time, target_velocities, 'r--', linewidth=2, label='Target Velocity')
    axes[0].plot(time, velocities, 'b-', linewidth=2, label='Actual Velocity')
    axes[0].set_ylabel('Velocity [m/s]')
    axes[0].set_title('PID Speed Control Performance')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    
    # 제어 출력
    axes[1].plot(time, control_outputs, 'g-', linewidth=2)
    axes[1].set_ylabel('Control Output')
    axes[1].set_title('PID Control Output (Pedal Input)')
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    
    # 오차
    axes[2].plot(time, errors, 'purple', linewidth=2)
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('Velocity Error [m/s]')
    axes[2].set_title('Velocity Tracking Error')
    axes[2].grid(True, alpha=0.3)
    axes[2].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    plt.show()
    
    # 성능 지표 계산
    rmse = np.sqrt(np.mean(np.array(errors)**2))
    max_error = np.max(np.abs(errors))
    settling_time = None
    
    # 정착 시간 계산 (5% 범위 내)
    for i in range(len(errors)-1, -1, -1):
        if abs(errors[i]) > 0.05 * target_velocities[i]:
            settling_time = time[i]
            break
    
    print("=== PID 제어기 성능 평가 ===")
    print(f"RMSE: {rmse:.3f} m/s")
    print(f"최대 오차: {max_error:.3f} m/s")
    if settling_time:
        print(f"정착 시간: {settling_time:.2f} s")
    else:
        print("정착 시간: < 0.1 s")

if __name__ == "__main__":
    test_pid_controller()
