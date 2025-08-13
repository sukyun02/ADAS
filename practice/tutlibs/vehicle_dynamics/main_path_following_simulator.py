"""
 * @file        main_path_following_simulator.py
 * @brief       Custom Path Following Simulator for vehicle dynamics
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)          
 *
 * @date        2025-08-14 Released by AI Lab, Hansung University
 * 
"""

# 필요한 라이브러리 임포트
import numpy as np

from custom_simviz import VehicleDynamicsSimulator, VehicleState, LongitudinalDynamics, LateralDynamics, animate_vehicle_trajectory
from custom_lat_pure_pursuit import PurePursuitController
from custom_long_pid import PIDController, AdaptiveSpeedController

# 경로 데이터 로드
def load_path_csv(csv_path):
    """CSV 경로 파일 로드"""
    try:
        data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
        return data[:, 0], data[:, 1]  # x, y 좌표
    except:
        data = np.loadtxt(csv_path, delimiter=',')
        return data[:, 0], data[:, 1]

# 통합 경로 추종 시뮬레이터 클래스
class PathFollowingSimulator:
    def __init__(self, vehicle_dynamics, lateral_controller, longitudinal_controller):
        self.vehicle_dynamics = vehicle_dynamics
        self.lateral_controller = lateral_controller
        self.longitudinal_controller = longitudinal_controller
        self.speed_controller = AdaptiveSpeedController(base_speed=12.0, curvature_factor=30.0)

    def calculate_path_yaw(self, path_x, path_y, closest_idx):
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

    def calculate_cross_track_error(self, state, path_x, path_y):
        """크로스 트랙 에러 계산 (부호 포함)"""
        # 가장 가까운 경로점에서 차량으로의 벡터
        vehicle_x = xxxxxx # TODO: 차량 x 좌표
        vehicle_y = xxxxxx # TODO: 차량 y 좌표
        distances = xxxxxx # TODO: 거리 계산식 작성
        closest_idx = np.argmin(distances)
        closest_x = xxxxxx # TODO: 가장 가까운 경로점 x 좌표
        closest_y = xxxxxx # TODO: 가장 가까운 경로점 y 좌표
        dx = vehicle_x - closest_x
        dy = vehicle_y - closest_y
        
        # 경로에 수직인 방향으로의 거리 (부호 포함)
        # 경로 방향에서 90도 회전한 벡터와의 내적
        path_yaw = self.calculate_path_yaw(path_x, path_y, closest_idx)
        cross_track_error = xxxxxx # TODO: 크로스 트랙 에러 계산식 작성
        return cross_track_error
        
    def find_closest_path_index(self, vehicle_x, vehicle_y, path_x, path_y):
        """가장 가까운 경로 인덱스 찾기"""
        distances = xxxxxx # TODO: 거리 계산식 작성
        return np.argmin(distances)
    
    def simulate_path_following(self, path_x, path_y, initial_state, duration=30.0, dt=0.02):
        """경로 추종 시뮬레이션"""
        time_steps = int(duration / dt)
        states = [initial_state]
        control_infos = []
        
        current_state = initial_state
        self.longitudinal_controller.reset()  # PID 초기화
        
        for step in range(time_steps):
            # 현재 위치에서 가장 가까운 경로점 찾기
            closest_idx = self.find_closest_path_index(
                current_state.x, current_state.y, path_x, path_y) # TODO: 가장 가까운 경로점 찾기
            
            # 목표 속도 계산 (곡률 기반)
            target_speed = self.speed_controller.get_target_speed(path_x, path_y, closest_idx) # TODO: 목표 속도 계산식 작성
            
            # 횡방향 제어 (조향각 계산)
            if isinstance(self.lateral_controller, PurePursuitController):
                steering_angle, lat_info = self.lateral_controller.pure_pursuit_control(
                    current_state.x, current_state.y, current_state.yaw, 
                    current_state.velocity, path_x, path_y) # TODO: Pure Pursuit 제어 계산식 작성
                
                # 룩어헤드 거리 계산
                lookahead_distance = self.lateral_controller.calculate_lookahead_distance(current_state.velocity)
                
                # 룩어헤드 포인트 찾기
                lookahead_point = self.lateral_controller.find_lookahead_point(
                    current_state.x, current_state.y, current_state.yaw, path_x, path_y, lookahead_distance) # TODO: 룩어헤드 포인트 찾기
            else:
                steering_angle, lat_info = 0.0, {}
            
            # 종방향 제어 (페달 입력 계산)
            pedal_input, long_info = self.longitudinal_controller.update(
                target_speed, current_state.velocity) # TODO: 종방향 제어 함수 완성
            
            # 차량 동역학 시뮬레이션
            new_state, sim_info = self.vehicle_dynamics.simulate_step(
                current_state, pedal_input, steering_angle, dt) # TODO: 차량 동역학 시뮬레이션 함수 완성
            
            # 경로 이탈 체크 (너무 멀리 벗어나면 시뮬레이션 종료)
            closest_distance = np.sqrt((new_state.x - path_x[closest_idx])**2 + 
                                     (new_state.y - path_y[closest_idx])**2)
            if closest_distance > 20.0:  # 20m 이상 벗어나면 종료
                print(f"경로 이탈로 시뮬레이션 종료 (step: {step}, distance: {closest_distance:.1f}m)")
                break
        
            # CTE 계산
            # 크로스 트랙 에러 계산
            cte = self.calculate_cross_track_error(new_state, path_x, path_y) # TODO: 크로스 트랙 에러 계산식 작성
            
            # 상태 업데이트
            current_state = new_state
            states.append(current_state)
            
            # 제어 정보 저장
            control_info = {
                'closest_idx': closest_idx,
                'lookahead_distance': lookahead_distance,
                'lookahead_point': lookahead_point,
                'target_speed': target_speed,
                'lateral_info': lat_info,
                'longitudinal_info': long_info,
                'steering_angle': steering_angle,
                'pedal_input': pedal_input,
                'cte': cte
            }
            control_infos.append(control_info)
            
            # 목표점 근처 도달 시 종료
            if closest_idx >= len(path_x) - 10:
                print(f"목표점 도달로 시뮬레이션 종료 (step: {step})")
                break
        
        return states, control_infos

def main():
    # 경로 로드
    path_x, path_y = load_path_csv("data/control_trajectory/racetrack_centerline.csv")
    # path_x, path_y = load_path_csv("data/control_trajectory/path_straight.csv")

    print("=== Pure Pursuit 및 PID 제어기 시뮬레이션 ===")

    # 시뮬레이터 생성
    long_dynamics = LongitudinalDynamics()
    lat_dynamics = LateralDynamics()
    simulator = VehicleDynamicsSimulator(long_dynamics, lat_dynamics)

    # 초기 상태 설정
    initial_state = VehicleState(
        x=path_x[-50], y=path_y[-50], yaw=-120 * np.pi / 180,
        velocity=5.0, beta=0.0, yaw_rate=0.0
    )

    # Pure Pursuit 시뮬레이션
    print("\nPure Pursuit 제어기 시뮬레이션 실행")
    pp_controller = PurePursuitController(wheelbase=lat_dynamics.L, lookahead_base=6.0, lookahead_gain=0.4)
    pid_controller_pp = PIDController(kp=2.0, ki=0.1, kd=0.05)
    simulator_pp = PathFollowingSimulator(simulator, pp_controller, pid_controller_pp)

    states_pp, infos_pp = simulator_pp.simulate_path_following(
        path_x, path_y, initial_state, duration=40.0, dt=0.02)

    print(f"Pure Pursuit 완료: {len(states_pp)}개 스텝")

    # 시각화
    fig, anim = animate_vehicle_trajectory(states_pp, dt=0.02, vehicle_length=4.5, vehicle_width=1.8, path_x=path_x, path_y=path_y, infos=infos_pp)
    return fig, anim

if __name__ == "__main__":
    main()