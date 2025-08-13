"""
 * @file        custom_simviz.py
 * @brief       Custom Vehicle Dynamics Models, Simulator and Visualizer for vehicle dynamics
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)          
 *
 * @date        2025-08-14 Released by AI Lab, Hansung University
 * 
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
from matplotlib import patches
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any

@dataclass
class VehicleState:
    """차량 상태 정의"""
    x: float = 0.0          # 위치 X [m]
    y: float = 0.0          # 위치 Y [m]  
    yaw: float = 0.0        # 요 각도 [rad]
    velocity: float = 0.0   # 속도 [m/s]
    beta: float = 0.0       # 슬립각 [rad]
    yaw_rate: float = 0.0   # 요 각속도 [rad/s]

# 종방향 동역학 모델 구현
class LongitudinalDynamics:
    def __init__(self):
        # 차량 파라미터
        self.m = 1300.0          # 차량 질량 [kg]
        self.I_m = 0.015         # 모터 관성 [kg⋅m²]
        self.I_t = 0.06          # 변속기 관성 [kg⋅m²]
        self.I_w = 0.3           # 휠 관성 [kg⋅m²]
        self.gear_ratio = 1.0 / 7.98  # 기어비
        self.r_eff = 0.30        # 휠 유효반지름 [m]
        
        # 공기역학 파라미터
        self.rho_air = 1.225     # 공기밀도 [kg/m³]
        self.Cd = 0.30           # 항력계수
        self.Af = 2.2            # 전면 면적 [m²]
        
        # 저항 파라미터
        self.k_R = 0.015         # 구름저항 계수
        self.slope = 0.0         # 경사각 [rad]
        self.g = 9.81            # 중력가속도 [m/s²]
        
        # 토크 제한
        self.motor_torque_max = 2000.0   # 최대 모터 토크 [Nm]
        self.brake_torque_max = 6000.0   # 최대 브레이크 토크 [Nm]

        self.accel_const = 300.0     # T_motor = accel_const * pedal_accel  [Nm]
        self.brake_const = 4000.0    # T_brake = brake_const * pedal_brake  [Nm]
        
    def calculate_equivalent_inertia(self):
        """등가 관성 계산"""
        J_eq = xxxxxx # TODO: 등가 관성 계산식 작성
        return J_eq
    
    def calculate_resistances(self, velocity):
        """저항력 계산"""
        # 부호 함수
        sgn = 1.0 if velocity >= 0 else -1.0
        
        # 구름저항
        F_roll = xxxxxx # TODO: 구름저항 계산식 작성
        
        # 공기저항
        F_aero = xxxxxx # TODO: 공기저항 계산식 작성
        
        # 중력저항 (경사)
        F_grav = xxxxxx # TODO: 중력저항 계산식 작성
        
        return F_roll, F_aero, F_grav
    
    def motor_dynamics(self, motor_torque, brake_torque, velocity):
        """모터 동역학 방정식"""
        # 등가 관성
        J_eq = self.xxxxxx() # TODO: 등가 관성 계산식 작성
        
        # 저항력들
        F_roll, F_aero, F_grav = self.xxxxxx(velocity) # TODO: 저항력 계산식 작성
        
        # 토크 제한
        T_m = xxxxxx # TODO: np.clip 사용하여 모터 토크 제한
        T_b = xxxxxx # TODO: np.clip 사용하여 브레이크 토크 제한
        
        # 로드 토크 (모터축 환산)
        T_load = xxxxxx # TODO: 로드 토크 계산식 작성
        
        # 모터 각가속도
        omega_dot = xxxxxx # TODO: 모터 각가속도 계산식 작성
        
        # 종가속도
        acceleration = xxxxxx # TODO: 종가속도 계산식 작성
        
        return acceleration, omega_dot, F_roll, F_aero, F_grav
    
    def pedal_to_torque(self, pedal_input):
        """페달 입력을 토크로 변환 (양수: 가속, 음수: 브레이크)"""
        if pedal_input >= 0:
            # 가속 페달
            motor_torque = pedal_input * xxxxxx # TODO: 가속 페달 토크 계산식 작성
            brake_torque = 0.0
        else:
            # 브레이크 페달
            motor_torque = 0.0
            brake_torque = -pedal_input * xxxxxx # TODO: 브레이크 페달 토크 계산식 작성
            
        return motor_torque, brake_torque

# 횡방향 동역학 모델 구현
class LateralDynamics:
    def __init__(self):
        # 차량 파라미터 (simviz_2.py 참고)
        self.m = 1300.0          # 차량 질량 [kg]
        self.Iz = 2100.0         # 요 관성 [kg⋅m²]
        self.Lf = 1.3            # 전축 거리 [m]
        self.Lr = 1.4            # 후축 거리 [m]
        self.L = self.Lf + self.Lr  # 휠베이스 [m]
        
        # 타이어 파라미터
        self.Cf = 160000.0       # 전륜 코너링 강성 [N/rad] (2*c_af)
        self.Cr = 170000.0       # 후륜 코너링 강성 [N/rad] (2*c_ar)
        self.max_alpha = np.deg2rad(7.0)  # 최대 슬립각 [rad]
        
        # 모델 스위칭 속도
        self.v_switch = 5.0      # kinematic ↔ dynamic 스위칭 속도 [m/s]
        
        # 제약 조건
        self.max_steer = np.deg2rad(30.0)  # 최대 조향각 [rad]
        self.beta_max = np.deg2rad(30.0)   # 최대 슬립각 [rad]
        self.yaw_rate_max = 2.0            # 최대 요 각속도 [rad/s]
        
    def kinematic_model(self, velocity, steering_angle, beta=0.0):
        """Kinematic 모델 (v < 5 m/s)"""
        # 조향각 제한
        delta = xxxxxx # TODO: np.clip 사용하여 조향각 제한 with max_steer
        
        # 슬립각 계산 (기하학적 관계)
        if abs(delta) < 1e-6:
            beta_new = 0.0
            yaw_rate = 0.0
        else:
            beta_new = xxxxxx # TODO: 슬립각 계산식 작성
            beta_new = xxxxxx # TODO: np.clip 사용하여 슬립각 제한
            
            # 요 각속도 계산
            yaw_rate = xxxxxx # TODO: 요 각속도 계산식 작성
            yaw_rate = xxxxxx # TODO: np.clip 사용하여 요 각속도 제한
        
        # 상태 미분값
        beta_dot = (beta_new - beta) * 50.0  # 부드러운 수렴을 위한 1차 시스템
        yaw_rate_dot = 0.0  # kinematic 모델에서는 요 각가속도 = 0
        
        return beta_dot, yaw_rate, yaw_rate_dot
    
    def dynamic_model(self, velocity, steering_angle, beta, yaw_rate):
        """Dynamic 모델 (v ≥ 5 m/s)"""
        # 조향각 제한
        delta = xxxxxx # TODO: np.clip 사용하여 조향각 제한 with max_steer
        
        # 속도 하한 (분모 보호)
        v = max(velocity, 0.1)
        
        # 슬립각 계산
        alpha_f = xxxxxx # TODO: 전륜 슬립각 계산식 작성
        alpha_r = xxxxxx # TODO: 후륜 슬립각 계산식 작성
        
        # 슬립각 제한
        alpha_f = xxxxxx # TODO: np.clip 사용하여 전륜 슬립각 제한 with max_alpha
        alpha_r = xxxxxx # TODO: np.clip 사용하여 후륜 슬립각 제한 with max_alpha
        
        # 측력 계산
        Fyf = xxxxxx # TODO: 전륜 측력 계산식 작성
        Fyr = xxxxxx # TODO: 후륜 측력 계산식 작성
        
        # 상태 미분값 계산
        beta_dot = xxxxxx # TODO: 슬립각 미분값 계산식 작성
        yaw_rate_dot = xxxxxx # TODO: 요 각속도 미분값 계산식 작성
        
        # 제한
        beta_dot = xxxxxx # TODO: np.clip 사용하여 슬립각 미분값 제한 (-10 ~ 10)
        yaw_rate_dot = xxxxxx # TODO: np.clip 사용하여 요 각속도 미분값 제한 (-10 ~ 10)
        
        return beta_dot, yaw_rate, yaw_rate_dot
    
    def lateral_dynamics(self, velocity, steering_angle, beta, yaw_rate):
        """속도에 따른 모델 자동 선택"""
        if velocity < xxxxxx: # TODO: v_switch 사용
            # Kinematic 모델 사용
            beta_dot, yaw_rate_new, yaw_rate_dot = self.kinematic_model(
                velocity, steering_angle, beta)
            model_type = "kinematic"
        else:
            # Dynamic 모델 사용
            beta_dot, yaw_rate_new, yaw_rate_dot = self.dynamic_model(
                velocity, steering_angle, beta, yaw_rate)
            model_type = "dynamic"
        
        return beta_dot, yaw_rate_new, yaw_rate_dot, model_type
    
    def lateral_dynamics_only_dynamic(self, velocity, steering_angle, beta, yaw_rate):
        # Dynamic 모델 사용
        beta_dot, yaw_rate_new, yaw_rate_dot = self.dynamic_model(
            velocity, steering_angle, beta, yaw_rate)
        model_type = "dynamic"
        
        return beta_dot, yaw_rate_new, yaw_rate_dot, model_type
    
    def lateral_dynamics_only_kinematic(self, velocity, steering_angle, beta, yaw_rate):
        # Kinematic 모델 사용
        beta_dot, yaw_rate_new, yaw_rate_dot = self.kinematic_model(
            velocity, steering_angle, beta)
        model_type = "kinematic"
        
        return beta_dot, yaw_rate_new, yaw_rate_dot, model_type
    
    def vehicle_kinematics(self, velocity, yaw_angle, beta):
        """차량 위치 운동학"""
        x_dot = xxxxxx # TODO: 종위치 미분값 계산식 작성
        y_dot = xxxxxx # TODO: 횡위치 미분값 계산식 작성
        return x_dot, y_dot

class VehicleDynamicsSimulator:
    """차량 동역학 통합 시뮬레이터"""
    
    def __init__(self, longitudinal_model, lateral_model):
        self.long_model = longitudinal_model
        self.lat_model = lateral_model
        
    def simulate_step(self, state: VehicleState, pedal_input: float, 
                     steering_input: float, dt: float) -> Tuple[VehicleState, Dict]:
        """한 스텝 시뮬레이션"""
        # 종방향 동역학
        motor_torque, brake_torque = self.long_model.xxxxxx(pedal_input) # TODO: 페달 입력을 토크로 변환 함수 작성
        acceleration, _, F_roll, F_aero, F_grav = self.long_model.xxxxxx(
            motor_torque, brake_torque, state.velocity) # TODO: 모터 동역학 방정식 함수 작성
        
        # 횡방향 동역학 (속도에 따른 모델 자동 선택)
        beta_dot, yaw_rate_new, yaw_rate_dot, model_type = self.lat_model.lateral_dynamics(
            state.velocity, steering_input, state.beta, state.yaw_rate) # TODO: 횡방향 동역학 함수 작성
        
        # 차량 위치 운동학
        x_dot, y_dot = self.lat_model.xxxxxx(
            state.velocity, state.yaw, state.beta) # TODO: 차량 위치 운동학 함수 작성
        
        # 상태 업데이트 (Euler integration)
        new_state = VehicleState(
            x = xxxxxx, # TODO: 종위치 계산식 작성
            y = xxxxxx, # TODO: 횡위치 계산식 작성
            yaw = xxxxxx, # TODO: 요 각도 계산식 작성
            velocity = max(0.0, xxxxxx), # TODO: 속도 계산식 작성
            beta = xxxxxx, # TODO: 슬립각 계산식 작성
            yaw_rate = xxxxxx # TODO: 요 각속도 계산식 작성
        )
        
        # 제한 적용
        new_state.beta = xxxxxx # TODO: np.clip 사용하여 슬립각 제한
        new_state.yaw_rate = xxxxxx # TODO: np.clip 사용하여 요 각속도 제한
        
        # 시뮬레이션 정보
        sim_info = {
            'acceleration': xxxxxx, # TODO: 가속도 계산식 작성
            'motor_torque': xxxxxx, # TODO: 모터 토크 계산식 작성
            'brake_torque': xxxxxx, # TODO: 브레이크 토크 계산식 작성
            'model_type': xxxxxx, # TODO: 모델 타입 계산식 작성
            'forces': {'roll': xxxxxx, 'aero': xxxxxx, 'grav': xxxxxx} # TODO: 저항력 계산식 작성
        }
        
        return new_state, sim_info
    
    def simulate_trajectory(self, initial_state: VehicleState, 
                          pedal_inputs: List[float], steering_inputs: List[float],
                          dt: float = 0.02) -> Tuple[List[VehicleState], List[Dict]]:
        """전체 궤적 시뮬레이션"""
        states = [initial_state]
        infos = []
        
        current_state = initial_state
        
        for pedal, steering in zip(pedal_inputs, steering_inputs):
            new_state, sim_info = self.simulate_step(current_state, pedal, steering, dt) # TODO: 시뮬레이션 실행 함수 완성
            states.append(new_state)
            infos.append(sim_info)
            current_state = new_state
            
        return states, infos

def plot_lateral_model_comparison(lat_model, velocities=[5.0, 10.0, 30.0], 
                                steering_angle=np.deg2rad(10.0), duration=5.0, dt=0.02):
    """속도별 횡방향 모델 비교 시각화"""
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('Lateral Model Comparison: Kinematic vs Dynamic', fontsize=16)
    
    colors = ['blue', 'red', 'green']
    
    for i, velocity in enumerate(velocities):
        # 시뮬레이션 설정
        time = np.arange(0, duration, dt)
        
        # 초기 상태
        state = VehicleState(velocity=velocity)
        state_k = VehicleState(velocity=velocity)
        
        # 데이터 저장
        positions_x, positions_y = [], []
        betas, yaw_rates = [], []
        model_types = []

        positions_x_k, positions_y_k = [], []
        betas_k, yaw_rates_k = [], []
        model_types_k = []
        
        # 시뮬레이션 실행
        for t in time:
            # 조향 입력 (2초 후 조향)
            steer = steering_angle if t > 2.0 else 0.0
            
            # 횡방향 동역학 계산 - Dynamic
            beta_dot, yaw_rate, yaw_rate_dot, model_type = lat_model.lateral_dynamics_only_dynamic(
                state.velocity, steer, state.beta, state.yaw_rate)
            
            # 위치 운동학
            x_dot, y_dot = lat_model.vehicle_kinematics(state.velocity, state.yaw, state.beta)
            
            # 상태 업데이트
            state.x += x_dot * dt
            state.y += y_dot * dt
            state.yaw += yaw_rate * dt
            state.beta += beta_dot * dt
            state.yaw_rate += yaw_rate_dot * dt
            
            # 데이터 저장
            positions_x.append(state.x)
            positions_y.append(state.y)
            betas.append(np.rad2deg(state.beta))
            yaw_rates.append(np.rad2deg(yaw_rate))
            model_types.append(model_type)

            # 횡방향 동역학 계산 - Kinematic
            beta_dot_k, yaw_rate_k, yaw_rate_dot_k, model_type_k = lat_model.lateral_dynamics_only_kinematic(
                state_k.velocity, steer, state_k.beta, state_k.yaw_rate)

            # 위치 운동학
            x_dot_k, y_dot_k = lat_model.vehicle_kinematics(state_k.velocity, state_k.yaw, state_k.beta)  

            # 상태 업데이트
            state_k.x += x_dot_k * dt
            state_k.y += y_dot_k * dt
            state_k.yaw += yaw_rate_k * dt
            state_k.beta += beta_dot_k * dt
            state_k.yaw_rate += yaw_rate_dot_k * dt

            positions_x_k.append(state_k.x)
            positions_y_k.append(state_k.y)
            betas_k.append(np.rad2deg(state_k.beta))
            yaw_rates_k.append(np.rad2deg(yaw_rate_k))
            model_types_k.append(model_type_k)

        # 궤적 플롯
        axes[0, i].plot(positions_x, positions_y, color=colors[1], linewidth=2, 
                       label=f'v = {velocity} m/s (Dynamic)', alpha=0.5)
        axes[0, i].plot(positions_x_k, positions_y_k, color=colors[0], linewidth=2, linestyle='--',
                       label=f'v = {velocity} m/s (Kinematic)', alpha=0.5)
        axes[0, i].set_xlabel('X [m]')
        axes[0, i].set_ylabel('Y [m]')
        axes[0, i].set_title(f'Trajectory (v = {velocity} m/s)')
        axes[0, i].grid(True, alpha=0.3)
        axes[0, i].axis('equal')
        axes[0, i].legend()
        
        # 모델 타입 표시
        model_type_text = model_types[0]
        model_type_text_k = model_types_k[0]
        axes[0, i].text(0.05, 0.95, f'Model: {model_type_text}', 
                       transform=axes[0, i].transAxes, 
                       bbox=dict(boxstyle="round,pad=0.3", facecolor=colors[1], alpha=0.7))
        axes[0, i].text(0.05, 0.85, f'Model: {model_type_text_k}', 
                       transform=axes[0, i].transAxes, 
                       bbox=dict(boxstyle="round,pad=0.3", facecolor=colors[0], alpha=0.7))

        # 슬립각 플롯
        axes[1, i].plot(time, betas, color=colors[1], linewidth=2, label=f'v = {velocity} m/s (Dynamic)', alpha=0.5)
        axes[1, i].plot(time, betas_k, color=colors[0], linewidth=2, linestyle='--', label=f'v = {velocity} m/s (Kinematic)', alpha=0.5)
        axes[1, i].set_xlabel('Time [s]')
        axes[1, i].set_ylabel('Slip Angle [deg]')
        axes[1, i].set_title(f'Slip Angle (v = {velocity} m/s)')
        axes[1, i].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def plot_vehicle_trajectory(states: List[VehicleState], infos: List[Dict], 
                          title: str = "Vehicle Trajectory"):
    """차량 궤적 및 상태 시각화"""
    
    # 데이터 추출
    time = np.arange(len(states)) * 0.02
    x_pos = [s.x for s in states]
    y_pos = [s.y for s in states]
    velocities = [s.velocity for s in states]
    betas = [np.rad2deg(s.beta) for s in states]
    yaw_rates = [np.rad2deg(s.yaw_rate) for s in states]
    
    if infos:
        accelerations = [info['acceleration'] for info in infos]
        model_types = [info['model_type'] for info in infos]
    else:
        accelerations = [0.0] * (len(states) - 1)
        model_types = ['unknown'] * (len(states) - 1)
    
    # 시각화
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle(title, fontsize=16)
    
    # 궤적
    axes[0, 0].plot(x_pos, y_pos, 'b-', linewidth=2)
    axes[0, 0].plot(x_pos[0], y_pos[0], 'go', markersize=8, label='Start')
    axes[0, 0].plot(x_pos[-1], y_pos[-1], 'ro', markersize=8, label='End')
    axes[0, 0].set_xlabel('X [m]')
    axes[0, 0].set_ylabel('Y [m]')
    axes[0, 0].set_title('Vehicle Trajectory')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axis('equal')
    axes[0, 0].legend()
    
    # 속도
    axes[0, 1].plot(time, velocities, 'r-', linewidth=2)
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Velocity [m/s]')
    axes[0, 1].set_title('Vehicle Velocity')
    axes[0, 1].grid(True, alpha=0.3)
    
    # 가속도
    if len(time) > 1:
        axes[0, 2].plot(time[1:], accelerations, 'g-', linewidth=2)
    axes[0, 2].set_xlabel('Time [s]')
    axes[0, 2].set_ylabel('Acceleration [m/s²]')
    axes[0, 2].set_title('Vehicle Acceleration')
    axes[0, 2].grid(True, alpha=0.3)
    
    # 슬립각
    axes[1, 0].plot(time, betas, 'purple', linewidth=2)
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Slip Angle [deg]')
    axes[1, 0].set_title('Vehicle Slip Angle')
    axes[1, 0].grid(True, alpha=0.3)
    
    # 요 각속도
    axes[1, 1].plot(time, yaw_rates, 'orange', linewidth=2)
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Yaw Rate [deg/s]')
    axes[1, 1].set_title('Vehicle Yaw Rate')
    axes[1, 1].grid(True, alpha=0.3)
    
    # 모델 타입 (히스토그램)
    if model_types:
        kinematic_count = model_types.count('kinematic')
        dynamic_count = model_types.count('dynamic')
        
        axes[1, 2].bar(['Kinematic', 'Dynamic'], [kinematic_count, dynamic_count], 
                      color=['blue', 'red'], alpha=0.7)
        axes[1, 2].set_ylabel('Usage Count')
        axes[1, 2].set_title('Model Usage')
        axes[1, 2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def animate_vehicle_trajectory(states: List[VehicleState], dt: float = 0.02, 
                             vehicle_length: float = 4.5, vehicle_width: float = 1.8, path_x: List[float] = None, path_y: List[float] = None, infos: List[Dict] = None):
    """차량 궤적 애니메이션"""
    
    # 데이터 추출
    x_pos = [s.x for s in states]
    y_pos = [s.y for s in states]
    yaw_angles = [s.yaw for s in states]
    
    # 플롯 설정
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    # 축 범위 설정
    x_min, x_max = min(x_pos) - 10, max(x_pos) + 10
    y_min, y_max = min(y_pos) - 10, max(y_pos) + 10
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    
    # 궤적 라인
    trajectory_line, = ax.plot([], [], 'b-', alpha=0.5, linewidth=1.5, label='Trajectory')
    
    # 차량 박스
    vehicle_patch = Rectangle((0, 0), vehicle_length, vehicle_width, 
                            fill=False, linewidth=2, color='blue')
    ax.add_patch(vehicle_patch)
    
    # 텍스트 정보
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                       bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                       verticalalignment='top')
                    
    # 경로 라인
    ax.plot(path_x, path_y, 'g--', alpha=0.7, linewidth=1, label='Path')
    lookahead_point, = ax.plot([], [], marker='o', markersize=5, linestyle='None', label="Lookahead", color='orange')

    def animate(frame):
        if frame < len(states):
            state = states[frame]
            if len(infos) > frame:
                log_info = infos[frame]
            else:
                log_info = infos[-1]
            
            # 궤적 업데이트
            trajectory_line.set_data(x_pos[:frame+1], y_pos[:frame+1])
            
            # 차량 위치 및 방향 업데이트
            x_center = state.x - vehicle_length / 2 * np.cos(state.yaw) + vehicle_width / 2 * np.sin(state.yaw)
            y_center = state.y - vehicle_length / 2 * np.sin(state.yaw) - vehicle_width / 2 * np.cos(state.yaw)
            
            # 차량 패치 변환
            t = patches.Affine2D().rotate(state.yaw).translate(x_center, y_center) + ax.transData
            vehicle_patch.set_transform(t)

            # lookahead 포인트 업데이트
            if log_info:
                lookahead_point.set_data([log_info['lookahead_point'][0]], [log_info['lookahead_point'][1]])
            else:
                lookahead_point.set_data([], [])
            
            # 정보 텍스트 업데이트
            info_text.set_text(f'Time: {frame * dt:.2f} s\n'
                              f'Velocity: {state.velocity:.2f} m/s\n'
                              f'Target Speed: {log_info["target_speed"]:.2f} m/s\n'
                              f'Heading: {np.rad2deg(state.yaw):.2f}°\n'
                              f'Slip Angle: {np.rad2deg(state.beta):.2f}°\n'
                              f'Yaw Rate: {np.rad2deg(state.yaw_rate):.2f}°/s\n'
                              f'Lookahead Distance: {log_info["lookahead_distance"]:.2f} m\n'
                              f'CTE: {log_info["cte"]:.2f} m')
        
        return trajectory_line, vehicle_patch, info_text, lookahead_point
    
    # 애니메이션 생성
    anim = FuncAnimation(fig, animate, frames=len(states), 
                        interval=int(dt*1000), blit=False, repeat=False)
    
    ax.legend()
    plt.title('Vehicle Dynamics Animation')
    plt.show()
    
    return fig, anim
