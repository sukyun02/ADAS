"""
 * @file        custom_kalman_filter.py
 * @brief       This module provides Kalman Filter implementation for vehicle motion estimation.
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
import time

class KalmanFilterVisualizer:
    """칼만 필터 결과 시각화 클래스"""
    
    def __init__(self):
        pass
        
    def plot_simulation_data(self, data: dict):
        """
        시뮬레이션 데이터 플롯
        
        Args:
            data: 시뮬레이션 데이터 딕셔너리
        """
        fig, axes = plt.subplots(6, 1, figsize=(12, 15))
        
        # 위치 X
        axes[0].plot(data['time'], data['position_x'], 'r-', linewidth=1.5)
        axes[0].set_xlabel('Time [s]')
        axes[0].set_ylabel('Position X [m]')
        axes[0].set_title('Position X')
        axes[0].grid(True)
        
        # 위치 Y
        axes[1].plot(data['time'], data['position_y'], 'r-', linewidth=1.5)
        axes[1].set_xlabel('Time [s]')
        axes[1].set_ylabel('Position Y [m]')
        axes[1].set_title('Position Y')
        axes[1].grid(True)
        
        # 속도 X
        axes[2].plot(data['time'], data['velocity_x'], 'g-', linewidth=1.5)
        axes[2].set_xlabel('Time [s]')
        axes[2].set_ylabel('Velocity X [m/s]')
        axes[2].set_title('Velocity X')
        axes[2].grid(True)
        
        # 속도 Y
        axes[3].plot(data['time'], data['velocity_y'], 'g-', linewidth=1.5)
        axes[3].set_xlabel('Time [s]')
        axes[3].set_ylabel('Velocity Y [m/s]')
        axes[3].set_title('Velocity Y')
        axes[3].grid(True)
        
        # 가속도 X
        axes[4].plot(data['time'], data['acceleration_x'], 'b-', linewidth=1.5)
        axes[4].set_xlabel('Time [s]')
        axes[4].set_ylabel('Acceleration X [m/s²]')
        axes[4].set_title('Acceleration X')
        axes[4].grid(True)
        
        # 가속도 Y
        axes[5].plot(data['time'], data['acceleration_y'], 'b-', linewidth=1.5)
        axes[5].set_xlabel('Time [s]')
        axes[5].set_ylabel('Acceleration Y [m/s²]')
        axes[5].set_title('Acceleration Y')
        axes[5].grid(True)
        
        plt.tight_layout()
        plt.show()
        
    def plot_trajectory_2d(self, data: dict):
        """
        2D 궤적 플롯
        
        Args:
            data: 시뮬레이션 데이터 딕셔너리
        """
        plt.figure(figsize=(10, 8))
        
        # 실제 궤적
        plt.plot(data['position_x'], data['position_y'], 'b-', linewidth=2, label='True Trajectory')
        
        # 인지 센서 측정값 (간격에 맞춰)
        interval = data['detection_update_interval']
        detection_indices = np.arange(0, len(data['time']), interval)
        plt.plot(data['detection_measurement_x'][detection_indices], 
                data['detection_measurement_y'][detection_indices], 
                'ko', markersize=6, label='Detection Measurements')
        
        plt.xlabel('Position X [m]')
        plt.ylabel('Position Y [m]')
        plt.title('Vehicle Trajectory in X-Y Plane')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        plt.show()
        
    def plot_estimation_results(self, data: dict, results: dict):
        """
        칼만 필터 추정 결과 플롯
        
        Args:
            data: 시뮬레이션 데이터
            results: 칼만 필터 결과
        """
        time = data['time']
        
        fig, axes = plt.subplots(4, 1, figsize=(12, 16))
        
        # 위치 X 추정
        axes[0].fill_between(time, 
                           results['x_propagation'][0, :] + results['std_propagation'][0, :],
                           results['x_propagation'][0, :] - results['std_propagation'][0, :],
                           alpha=0.3, color='red', label='Propagation 1σ')
        axes[0].fill_between(time,
                           results['x_kalman'][0, :] + results['std_kalman'][0, :],
                           results['x_kalman'][0, :] - results['std_kalman'][0, :],
                           alpha=0.6, color='yellow', label='Kalman Filter 1σ')
        axes[0].plot(time, results['x_propagation'][0, :], '--r', linewidth=2, label='Propagation')
        axes[0].plot(time, results['x_kalman'][0, :], '--b', linewidth=2, label='Kalman Filter')
        axes[0].plot(time, data['position_x'], '-k', linewidth=1, label='Ground Truth')
        axes[0].set_xlabel('Time [s]')
        axes[0].set_ylabel('Position X [m]')
        axes[0].set_title('Estimated Position X')
        axes[0].grid(True)
        axes[0].legend()
        
        # 위치 Y 추정
        axes[1].fill_between(time,
                           results['x_propagation'][1, :] + results['std_propagation'][1, :],
                           results['x_propagation'][1, :] - results['std_propagation'][1, :],
                           alpha=0.3, color='red', label='Propagation 1σ')
        axes[1].fill_between(time,
                           results['x_kalman'][1, :] + results['std_kalman'][1, :],
                           results['x_kalman'][1, :] - results['std_kalman'][1, :],
                           alpha=0.6, color='yellow', label='Kalman Filter 1σ')
        axes[1].plot(time, results['x_propagation'][1, :], '--r', linewidth=2, label='Propagation')
        axes[1].plot(time, results['x_kalman'][1, :], '--b', linewidth=2, label='Kalman Filter')
        axes[1].plot(time, data['position_y'], '-k', linewidth=1, label='Ground Truth')
        axes[1].set_xlabel('Time [s]')
        axes[1].set_ylabel('Position Y [m]')
        axes[1].set_title('Estimated Position Y')
        axes[1].grid(True)
        axes[1].legend()
        
        # 속도 X 추정
        axes[2].fill_between(time,
                           results['x_propagation'][2, :] + results['std_propagation'][2, :],
                           results['x_propagation'][2, :] - results['std_propagation'][2, :],
                           alpha=0.3, color='lightgreen', label='Propagation 1σ')
        axes[2].fill_between(time,
                           results['x_kalman'][2, :] + results['std_kalman'][2, :],
                           results['x_kalman'][2, :] - results['std_kalman'][2, :],
                           alpha=0.6, color='lightblue', label='Kalman Filter 1σ')
        axes[2].plot(time, results['x_propagation'][2, :], '--g', linewidth=2, label='Propagation')
        axes[2].plot(time, results['x_kalman'][2, :], '--b', linewidth=2, label='Kalman Filter')
        axes[2].plot(time, data['velocity_x'], '-k', linewidth=1, label='Ground Truth')
        axes[2].set_xlabel('Time [s]')
        axes[2].set_ylabel('Velocity X [m/s]')
        axes[2].set_title('Estimated Velocity X')
        axes[2].grid(True)
        axes[2].legend()
        
        # 속도 Y 추정
        axes[3].fill_between(time,
                           results['x_propagation'][3, :] + results['std_propagation'][3, :],
                           results['x_propagation'][3, :] - results['std_propagation'][3, :],
                           alpha=0.3, color='lightgreen', label='Propagation 1σ')
        axes[3].fill_between(time,
                           results['x_kalman'][3, :] + results['std_kalman'][3, :],
                           results['x_kalman'][3, :] - results['std_kalman'][3, :],
                           alpha=0.6, color='lightblue', label='Kalman Filter 1σ')
        axes[3].plot(time, results['x_propagation'][3, :], '--g', linewidth=2, label='Propagation')
        axes[3].plot(time, results['x_kalman'][3, :], '--b', linewidth=2, label='Kalman Filter')
        axes[3].plot(time, data['velocity_y'], '-k', linewidth=1, label='Ground Truth')
        axes[3].set_xlabel('Time [s]')
        axes[3].set_ylabel('Velocity Y [m/s]')
        axes[3].set_title('Estimated Velocity Y')
        axes[3].grid(True)
        axes[3].legend()
        
        plt.tight_layout()
        plt.show()
        
    def plot_covariance_evolution(self, data: dict, results: dict):
        """
        공분산 진화 플롯
        
        Args:
            data: 시뮬레이션 데이터
            results: 칼만 필터 결과
        """
        time = data['time']
        
        plt.figure(figsize=(12, 6))
        plt.semilogy(time, results['std_propagation'][0, :]**2, 'r-', linewidth=2, 
                    label='Propagation Position X Covariance')
        plt.semilogy(time, results['std_propagation'][2, :]**2, 'g-', linewidth=2,
                    label='Propagation Velocity X Covariance')
        plt.semilogy(time, results['std_kalman'][0, :]**2, 'b-', linewidth=2,
                    label='Kalman Filter Position X Covariance')
        plt.semilogy(time, results['std_kalman'][2, :]**2, 'c-', linewidth=2,
                    label='Kalman Filter Velocity X Covariance')
        plt.xlabel('Time [s]')
        plt.ylabel('Covariance')
        plt.title('Covariance Evolution of Estimated States')
        plt.grid(True)
        plt.legend()
        plt.show()
        
    def plot_covariance_ellipse(self, center: np.ndarray, cov_matrix: np.ndarray, 
                              n_sigma: float = 1.0, **kwargs) -> Ellipse:
        """
        공분산 타원 플롯
        
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
    
    def create_kalman_filter_animation(self, data: dict, results: dict, 
                                     save_path: str = None, fps: int = 10,
                                     figsize: tuple = (12, 10)) -> None:
        """
        칼만 필터 결과를 애니메이션으로 시각화
        
        Args:
            data: 시뮬레이션 데이터
            results: 칼만 필터 결과
            save_path: 저장 경로 (None이면 화면에만 표시)
            fps: 초당 프레임 수
            figsize: 그림 크기
        """
        # 애니메이션 설정
        time_data = data['time']
        n_frames = len(time_data)
        
        # 그림 초기화
        fig, ax = plt.subplots(figsize=figsize)
        
        # 전체 궤적 미리 그리기 (연한 색으로)
        ax.plot(data['position_x'], data['position_y'], 'k-', 
                linewidth=1, alpha=0.3, label='Ground Truth Path')
        
        # 칼만 필터 전체 궤적 (연한 색으로)
        ax.plot(results['x_kalman'][0, :], results['x_kalman'][1, :], 
                'b--', linewidth=1, alpha=0.3, label='Kalman Filter Path')
        
        # 인지 센서 측정값 미리 표시
        interval = data['detection_update_interval']
        detection_indices = np.arange(0, len(time_data), interval)
        ax.plot(data['detection_measurement_x'][detection_indices], 
                data['detection_measurement_y'][detection_indices], 
                'ro', markersize=3, alpha=0.3, label='Detection Measurements')
        
        # 애니메이션 요소 초기화
        current_pos_gt, = ax.plot([], [], 'ko', markersize=8, 
                                 markerfacecolor='black', markeredgecolor='white',
                                 markeredgewidth=2, label='Current GT Position')
        current_pos_kf, = ax.plot([], [], 'bo', markersize=8,
                                 markerfacecolor='blue', markeredgecolor='white', 
                                 markeredgewidth=2, label='Current KF Position')
        
        # 속도 화살표 초기화
        velocity_arrow_gt = ax.annotate('', xy=(0, 0), xytext=(0, 0),
                                       arrowprops=dict(arrowstyle='->', color='green', lw=2))
        velocity_arrow_kf = ax.annotate('', xy=(0, 0), xytext=(0, 0),
                                       arrowprops=dict(arrowstyle='->', color='blue', lw=2))
        
        # 공분산 타원 초기화
        ellipse_patch = None
        
        # 궤적 추적선 초기화
        trail_gt, = ax.plot([], [], 'k-', linewidth=2, alpha=0.8)
        trail_kf, = ax.plot([], [], 'b-', linewidth=2, alpha=0.8)
        
        # 시간 텍스트
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                           fontsize=14, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        # 축 설정
        margin = 5
        x_min, x_max = np.min(data['position_x']) - margin, np.max(data['position_x']) + margin
        y_min, y_max = np.min(data['position_y']) - margin, np.max(data['position_y']) + margin
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel('Position X [m]')
        ax.set_ylabel('Position Y [m]')
        ax.set_title('Kalman Filter Animation: Vehicle Tracking with Covariance')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        ax.set_aspect('equal')
        
        def animate(frame):
            nonlocal ellipse_patch
            
            # 현재 시간
            current_time = time_data[frame]
            
            # Ground Truth 현재 위치
            gt_x, gt_y = data['position_x'][frame], data['position_y'][frame]
            gt_vx, gt_vy = data['velocity_x'][frame], data['velocity_y'][frame]
            
            # Kalman Filter 현재 위치
            kf_x, kf_y = results['x_kalman'][0, frame], results['x_kalman'][1, frame]
            kf_vx, kf_vy = results['x_kalman'][2, frame], results['x_kalman'][3, frame]
            
            # 현재 위치 업데이트
            current_pos_gt.set_data([gt_x], [gt_y])
            current_pos_kf.set_data([kf_x], [kf_y])
            
            # 속도 화살표 업데이트 (크기 조정)
            scale = 0.5  # 화살표 크기 조정
            velocity_arrow_gt.set_position((gt_x, gt_y))
            velocity_arrow_gt.xy = (gt_x + gt_vx * scale, gt_y + gt_vy * scale)
            
            velocity_arrow_kf.set_position((kf_x, kf_y))
            velocity_arrow_kf.xy = (kf_x + kf_vx * scale, kf_y + kf_vy * scale)
            
            # 궤적 추적선 업데이트 (현재까지의 경로)
            trail_gt.set_data(data['position_x'][:frame+1], data['position_y'][:frame+1])
            trail_kf.set_data(results['x_kalman'][0, :frame+1], results['x_kalman'][1, :frame+1])
            
            # 공분산 타원 업데이트
            if ellipse_patch is not None:
                ellipse_patch.remove()
            
            center = np.array([kf_x, kf_y])
            cov_matrix = results['P_kalman'][:2, :2, frame]
            ellipse_patch = self.plot_covariance_ellipse(
                center, cov_matrix, n_sigma=1.0,
                facecolor='yellow', edgecolor='orange', alpha=0.6
            )
            ax.add_patch(ellipse_patch)
            
            # 시간 텍스트 업데이트
            time_text.set_text(f'Time: {current_time:.2f}s\nFrame: {frame+1}/{n_frames}')
            
            # 측정 업데이트 시점 표시
            if frame % interval == 0:
                # 측정 업데이트 시점에 특별한 표시
                current_pos_kf.set_markeredgecolor('yellow')
                current_pos_kf.set_markeredgewidth(4)
            else:
                current_pos_kf.set_markeredgecolor('white')
                current_pos_kf.set_markeredgewidth(2)
            
            return (current_pos_gt, current_pos_kf, velocity_arrow_gt, velocity_arrow_kf,
                   trail_gt, trail_kf, ellipse_patch, time_text)
        
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
            else:
                print("⚠️ 지원되는 형식: .gif, .mp4")
            print("✅ 애니메이션 저장 완료!")
        else:
            plt.show()
        
        return anim
    
    def create_simple_animation(self, data: dict, results: dict, 
                              interval_ms: int = 100, figsize: tuple = (12, 10)) -> None:
        """
        간단한 실시간 스타일 애니메이션 (저장 없이 화면 표시용)
        
        Args:
            data: 시뮬레이션 데이터
            results: 칼만 필터 결과  
            interval_ms: 프레임 간 간격 (밀리초)
            figsize: 그림 크기
        """
        time_data = data['time']
        n_frames = len(time_data)
        
        # 그림 설정
        plt.ion()  # 인터랙티브 모드 활성화
        fig, ax = plt.subplots(figsize=figsize)
        
        # 전체 경로 미리 그리기 (연한 색)
        ax.plot(data['position_x'], data['position_y'], 'k-', 
                linewidth=1, alpha=0.2, label='GT Path')
        ax.plot(results['x_kalman'][0, :], results['x_kalman'][1, :], 
                'b--', linewidth=1, alpha=0.2, label='KF Path')
        
        # 축 설정
        margin = 3
        x_min, x_max = np.min(data['position_x']) - margin, np.max(data['position_x']) + margin
        y_min, y_max = np.min(data['position_y']) - margin, np.max(data['position_y']) + margin
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel('Position X [m]')
        ax.set_ylabel('Position Y [m]')
        ax.set_title('Real-time Kalman Filter Visualization')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_aspect('equal')
        
        print("실시간 애니메이션 시작... (창을 닫으면 종료)")
        
        # 각 프레임을 순차적으로 표시
        for frame in range(n_frames):
            ax.clear()
            
            # 기본 설정 다시 적용
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
            ax.set_xlabel('Position X [m]')
            ax.set_ylabel('Position Y [m]')
            ax.set_title(f'Kalman Filter - Time: {time_data[frame]:.2f}s')
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
            
            # 전체 경로 (연한 색)
            ax.plot(data['position_x'], data['position_y'], 'k-', 
                   linewidth=1, alpha=0.2, label='GT Full Path')
            ax.plot(results['x_kalman'][0, :], results['x_kalman'][1, :], 
                   'b--', linewidth=1, alpha=0.2, label='KF Full Path')
            
            # 현재까지의 궤적 (진한 색)
            ax.plot(data['position_x'][:frame+1], data['position_y'][:frame+1], 
                   'k-', linewidth=2, label='GT Trail')
            ax.plot(results['x_kalman'][0, :frame+1], results['x_kalman'][1, :frame+1], 
                   'b-', linewidth=2, label='KF Trail')
            
            # 현재 위치
            gt_x, gt_y = data['position_x'][frame], data['position_y'][frame]
            kf_x, kf_y = results['x_kalman'][0, frame], results['x_kalman'][1, frame]
            
            ax.plot(gt_x, gt_y, 'ko', markersize=10, markerfacecolor='green',
                   markeredgecolor='white', markeredgewidth=2, label='GT Current')
            ax.plot(kf_x, kf_y, 'bo', markersize=10, markerfacecolor='blue', 
                   markeredgecolor='white', markeredgewidth=2, label='KF Current')
            
            # 속도 화살표
            scale = 0.5
            gt_vx, gt_vy = data['velocity_x'][frame], data['velocity_y'][frame]
            kf_vx, kf_vy = results['x_kalman'][2, frame], results['x_kalman'][3, frame]
            
            ax.arrow(gt_x, gt_y, gt_vx * scale, gt_vy * scale,
                    head_width=0.5, head_length=0.3, fc='green', ec='green', alpha=0.8)
            ax.arrow(kf_x, kf_y, kf_vx * scale, kf_vy * scale,
                    head_width=0.5, head_length=0.3, fc='blue', ec='blue', alpha=0.8)
            
            # 공분산 타원
            center = np.array([kf_x, kf_y])
            cov_matrix = results['P_kalman'][:2, :2, frame]
            ellipse = self.plot_covariance_ellipse(
                center, cov_matrix, n_sigma=1.0,
                facecolor='yellow', edgecolor='orange', alpha=0.5
            )
            ax.add_patch(ellipse)
            
            # 측정 시점 표시
            interval = data['detection_update_interval']
            if frame % interval == 0:
                ax.plot(data['detection_measurement_x'][frame], 
                       data['detection_measurement_y'][frame],
                       'rs', markersize=8, alpha=0.8, label='Detection')
            
            ax.legend(loc='upper right', fontsize=8)
            
            # 화면 업데이트
            plt.draw()
            plt.pause(interval_ms / 1000.0)
            
            # 창이 닫혔는지 확인
            if not plt.get_fignums():
                break
        
        plt.ioff()  # 인터랙티브 모드 비활성화
        print("애니메이션 완료!")