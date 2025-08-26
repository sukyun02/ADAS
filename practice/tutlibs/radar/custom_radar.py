"""Radar Practice"""
"""
 * @file        custom_radar.py
 * @brief       Custom Radar practice algorithms for radar data processing
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)          
 *
 * @date        2025-08-12 Released by AI Lab, Hanyang University
 * 
"""
import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import glob
import os
from typing import List, Dict, Tuple, Optional, Union, Any
import time
from scipy.spatial.distance import cdist
from sklearn.cluster import DBSCAN
import copy

###############################################################################
# 1. Radar Data Loader
class RadarDataLoader:
    """레이더 포인트클라우드 데이터 로딩 클래스"""
    
    def __init__(self):
        self.point_clouds = []
        self.timestamps = []
        self.vehicle_speeds = []
        self.point_velocities = []  # 각 포인트의 도플러 속도 저장
        
    def load_radar_pcd_files(self, pcd_folder_path: str, pattern: str = "*.pcd") -> List[o3d.geometry.PointCloud]:
        """
        레이더 PCD 파일들을 로딩
        
        Args:
            pcd_folder_path: PCD 파일들이 있는 폴더 경로
            pattern: 파일 패턴 (기본값: "*.pcd")
            
        Returns:
            로딩된 포인트클라우드 리스트
        """
        pcd_files = sorted(glob.glob(os.path.join(pcd_folder_path, pattern)))
        
        if not pcd_files:
            print(f"⚠️ {pcd_folder_path}에서 PCD 파일을 찾을 수 없습니다.")
            print("시뮬레이션 데이터를 생성합니다...")
            return self._generate_simulation_radar_data()
        
        print(f"📡 {len(pcd_files)}개의 레이더 PCD 파일을 로딩 중...")
        
        for i, pcd_file in enumerate(pcd_files):
            try:
                # 커스텀 파서로 velocity field 포함하여 로딩
                pcd, velocities = self._load_pcd_with_velocity(pcd_file)
                if len(pcd.points) > 0:
                    self.point_clouds.append(pcd)
                    self.point_velocities.append(velocities)
                    self.timestamps.append(i * 0.1)  # 0.1초 간격
                    
                    if i % 10 == 0:
                        print(f"로딩 진행률: {i+1}/{len(pcd_files)} ({(i+1)/len(pcd_files)*100:.1f}%)")
                        print(f"   velocity field: {'✅' if len(velocities) > 0 else '❌'}")
                        
            except Exception as e:
                print(f"⚠️ {pcd_file} 로딩 실패: {e}")
                continue
        
        print(f"✅ {len(self.point_clouds)}개의 포인트클라우드 로딩 완료")
        return self.point_clouds
    
    def load_vehicle_speed_data(self, speed_file_path: str) -> List[float]:
        """
        차량 속도 데이터 로딩
        
        Args:
            speed_file_path: 속도 데이터 파일 경로 (CSV 또는 TXT)
            
        Returns:
            차량 속도 리스트 [m/s]
        """
        try:
            if speed_file_path.endswith('.csv'):
                speed_data = pd.read_csv(speed_file_path)
                # 첫 번째 열을 속도로 가정
                speeds = speed_data.iloc[:, 0].values
            else:  # txt 파일
                speeds = np.loadtxt(speed_file_path)
            
            # 포인트클라우드 개수와 맞춤
            if len(speeds) != len(self.point_clouds):
                print(f"⚠️ 속도 데이터 개수({len(speeds)})와 포인트클라우드 개수({len(self.point_clouds)})가 다릅니다.")
                speeds = np.resize(speeds, len(self.point_clouds))
            
            self.vehicle_speeds = speeds.tolist()
            print(f"✅ 차량 속도 데이터 로딩 완료: {len(self.vehicle_speeds)}개")
            
        except Exception as e:
            print(f"⚠️ 속도 파일 로딩 실패: {e}")
            print("모든 속도를 0으로 설정합니다.")
            self.vehicle_speeds = [0.0] * len(self.point_clouds)
        
        return self.vehicle_speeds
    
    def _load_pcd_with_velocity(self, pcd_file: str) -> tuple:
        """
        PCD 파일에서 velocity field를 포함하여 로딩
        
        Args:
            pcd_file: PCD 파일 경로
            
        Returns:
            (point_cloud, velocities) 튜플
        """
        try:
            # 1. 헤더 파싱
            with open(pcd_file, 'r') as f:
                lines = f.readlines()
            
            header_info = {}
            data_start_idx = 0
            
            for i, line in enumerate(lines):
                line = line.strip()
                if line.startswith('FIELDS'):
                    header_info['fields'] = line.split()[1:]
                elif line.startswith('SIZE'):
                    header_info['sizes'] = [int(x) for x in line.split()[1:]]
                elif line.startswith('TYPE'):
                    header_info['types'] = line.split()[1:]
                elif line.startswith('COUNT'):
                    header_info['counts'] = [int(x) for x in line.split()[1:]]
                elif line.startswith('POINTS'):
                    header_info['points'] = int(line.split()[1])
                elif line.startswith('DATA'):
                    header_info['data_type'] = line.split()[1]
                    data_start_idx = i + 1
                    break
            
            # 2. field 인덱스 찾기
            x_idx = header_info['fields'].index('x') if 'x' in header_info['fields'] else None
            y_idx = header_info['fields'].index('y') if 'y' in header_info['fields'] else None  
            z_idx = header_info['fields'].index('z') if 'z' in header_info['fields'] else None
            vel_idx = None
            
            if 'vel' in header_info['fields']:
                vel_idx = header_info['fields'].index('vel')
            elif 'velocity' in header_info['fields']:
                vel_idx = header_info['fields'].index('velocity')
            
            if x_idx is None or y_idx is None or z_idx is None:
                raise ValueError("x, y, z 필드를 찾을 수 없습니다.")
            
            # 3. 데이터 읽기
            points = []
            velocities = []
            
            if header_info['data_type'] == 'ascii':
                for line in lines[data_start_idx:]:
                    if line.strip():
                        values = line.strip().split()
                        if len(values) >= max(x_idx, y_idx, z_idx) + 1:
                            try:
                                x = float(values[x_idx])
                                y = float(values[y_idx])
                                z = float(values[z_idx])
                                points.append([x, y, z])
                                
                                if vel_idx is not None and len(values) > vel_idx:
                                    vel = float(values[vel_idx])
                                    velocities.append(vel)
                                else:
                                    velocities.append(0.0)  # 기본값
                                    
                            except (ValueError, IndexError):
                                continue
            else:
                # 바이너리 데이터는 일단 Open3D로 fallback
                print(f"⚠️ 바이너리 PCD는 Open3D로 fallback: {pcd_file}")
                pcd = o3d.io.read_point_cloud(pcd_file)
                points = np.asarray(pcd.points)
                velocities = [0.0] * len(points)  # velocity 정보 없음
            
            # 4. Open3D PointCloud 생성
            pcd = o3d.geometry.PointCloud()
            if points:
                pcd.points = o3d.utility.Vector3dVector(np.array(points))
                
                # velocity를 색상으로 시각화 (선택적)
                if velocities:
                    vel_array = np.array(velocities)
                    # velocity 정규화 (-20 ~ +20 m/s 범위)
                    normalized_vel = np.clip((vel_array + 20) / 40, 0, 1)
                    # colors = plt.cm.gist_rainbow(normalized_vel)[:, :3]  # RGB만 사용
                    colors = plt.cm.brg(normalized_vel)[:, :3]  # RGB만 사용
                    pcd.colors = o3d.utility.Vector3dVector(colors)
            
            return pcd, np.array(velocities) if velocities else np.array([])
            
        except Exception as e:
            print(f"⚠️ 커스텀 PCD 로딩 실패, Open3D로 fallback: {e}")
            # Open3D로 fallback
            pcd = o3d.io.read_point_cloud(pcd_file)
            velocities = np.zeros(len(pcd.points))  # velocity 정보 없음
            return pcd, velocities
    
    def _generate_simulation_radar_data(self) -> List[o3d.geometry.PointCloud]:
        """시뮬레이션 레이더 데이터 생성"""
        print("🔧 시뮬레이션 레이더 데이터 생성 중...")
        
        n_frames = 50
        point_clouds = []
        
        # 고정 객체들 (정적)
        static_objects = [
            np.array([10, 5, 0]),   # 우측 정적 객체
            np.array([15, -3, 0]),  # 좌측 정적 객체
            np.array([25, 0, 0]),   # 전방 정적 객체
        ]
        
        # 이동 객체들 (동적)
        for frame_idx in range(n_frames):
            pcd = o3d.geometry.PointCloud()
            points = []
            velocities = []
            
            # 정적 객체들 추가
            for static_obj in static_objects:
                # 각 객체 주변에 노이즈 포인트 추가
                noise_points = static_obj + np.random.normal(0, 0.5, (5, 3))
                points.extend(noise_points)
                velocities.extend([0.0] * 5)  # 정적 객체는 속도 0
            
            # 동적 객체 1: 우측에서 좌측으로 이동
            dynamic1_x = 20 - frame_idx * 0.3
            dynamic1_y = 8
            if dynamic1_x > -5:  # 시야 범위 내
                dynamic1_points = np.array([dynamic1_x, dynamic1_y, 0]) + np.random.normal(0, 0.3, (3, 3))
                points.extend(dynamic1_points)
                velocities.extend([-3.0] * 3)  # -3 m/s 속도
            
            # 동적 객체 2: 좌측에서 우측으로 이동
            dynamic2_x = -5 + frame_idx * 0.4
            dynamic2_y = -6
            if dynamic2_x < 30:  # 시야 범위 내
                dynamic2_points = np.array([dynamic2_x, dynamic2_y, 0]) + np.random.normal(0, 0.3, (4, 3))
                points.extend(dynamic2_points)
                velocities.extend([4.0] * 4)  # +4 m/s 속도
            
            # 랜덤 노이즈 포인트들
            noise_points = np.random.uniform([-5, -10, -1], [30, 10, 1], (10, 3))
            points.extend(noise_points)
            velocities.extend(np.random.normal(0, 1, 10))  # 랜덤 속도
            
            # 포인트클라우드 생성
            if points:
                pcd.points = o3d.utility.Vector3dVector(np.array(points))
                
                # 속도 정보를 컬러로 매핑 (도플러 속도)
                vel_array = np.array(velocities)
                # 속도 정규화 (-5 ~ +5 m/s 범위)
                normalized_vel = np.clip((vel_array + 5) / 10, 0, 1)
                colors = plt.cm.RdYlBu_r(normalized_vel)[:, :3]  # RGB만 사용
                pcd.colors = o3d.utility.Vector3dVector(colors)
                
                # 속도 정보를 사용자 정의 속성으로 저장
                # Open3D에서는 직접적인 custom attribute 지원이 제한적이므로
                # 별도로 관리하거나 color를 통해 역산
                
            point_clouds.append(pcd)
            self.timestamps.append(frame_idx * 0.1)
        
        self.point_clouds = point_clouds
        self.vehicle_speeds = [5.0] * n_frames  # 5 m/s 일정 속도
        
        # 시뮬레이션 데이터에서 velocity 정보 생성 (색상에서 역산)
        simulation_velocities = []
        for pcd in point_clouds:
            if pcd.has_colors():
                colors = np.asarray(pcd.colors)
                # RdYlBu_r 컬러맵에서 속도 역산 (-5 ~ +5 m/s 범위)
                velocities = (1 - colors[:, 0]) * 10 - 5
                simulation_velocities.append(velocities)
            else:
                simulation_velocities.append(np.zeros(len(pcd.points)))
        
        self.point_velocities = simulation_velocities
        
        print(f"✅ {len(point_clouds)}개의 시뮬레이션 포인트클라우드 생성 완료")
        return point_clouds

class RadarVisualizer:
    """레이더 데이터 시각화 클래스"""
    
    def __init__(self):
        self.vis = None

    def visualize_single_frame(self, pcd: o3d.geometry.PointCloud, frame_idx: int = 0, visualize_coordinate_frame: bool = True):
        """단일 프레임 포인트클라우드 시각화"""
        print(f"📊 프레임 {frame_idx} 시각화 중...")
        
        # 좌표축 추가
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0)
        if visualize_coordinate_frame:
            geometries = [pcd, coordinate_frame]
        else:
            geometries = [pcd]

        # # point cloud의 color map 지정
        # colors = np.asarray(pcd.colors)
        # colors = plt.cm.hsv(colors)
        # pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # 시각화
        o3d.visualization.draw_geometries(
            geometries,
            window_name=f"Radar Point Cloud - Frame {frame_idx}",
            width=1200,
            height=800,
            left=50,
            top=50
        )
    
    def visualize_accumulated_frames(self, point_clouds: List[o3d.geometry.PointCloud], 
                                   max_frames: int = None):
        """누적된 포인트클라우드 시각화"""
        if max_frames is None:
            max_frames = len(point_clouds)
        
        print(f"📊 {min(max_frames, len(point_clouds))}개 프레임 누적 시각화 중...")
        
        # 모든 포인트 누적
        accumulated_points = []
        accumulated_colors = []
        
        for i, pcd in enumerate(point_clouds[:max_frames]):
            if len(pcd.points) > 0:
                points = np.asarray(pcd.points)
                colors = np.asarray(pcd.colors) if pcd.has_colors() else np.ones((len(points), 3)) * 0.5
                
                # 시간에 따른 투명도 효과 (최신이 더 진함)
                alpha = 0.3 + 0.7 * (i / max_frames)
                colors = colors * alpha
                
                accumulated_points.append(points)
                accumulated_colors.append(colors)
        
        if accumulated_points:
            # 누적 포인트클라우드 생성
            all_points = np.vstack(accumulated_points)
            all_colors = np.vstack(accumulated_colors)
            
            accumulated_pcd = o3d.geometry.PointCloud()
            accumulated_pcd.points = o3d.utility.Vector3dVector(all_points)
            accumulated_pcd.colors = o3d.utility.Vector3dVector(all_colors)
            
            # 좌표축 추가
            coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0)
            
            # 시각화
            o3d.visualization.draw_geometries(
                [accumulated_pcd, coordinate_frame],
                window_name=f"Accumulated Radar Point Cloud ({max_frames} frames)",
                width=1200,
                height=800,
                left=50,
                top=50
            )
    
    def create_animation(self, point_clouds: List[o3d.geometry.PointCloud], 
                        timestamps: List[float], 
                        save_path: str = None,
                        fps: int = 10):
        """포인트클라우드 애니메이션 생성"""
        print(f"🎬 포인트클라우드 애니메이션 생성 중... ({len(point_clouds)}개 프레임)")
        
        # Open3D 애니메이션은 복잡하므로 matplotlib 기반으로 2D 시각화
        fig, ax = plt.subplots(figsize=(24, 8))
        
        def animate(frame_idx):
            ax.clear()
            
            if frame_idx < len(point_clouds):
                pcd = point_clouds[frame_idx]
                if len(pcd.points) > 0:
                    points = np.asarray(pcd.points)
                    colors = np.asarray(pcd.colors) if pcd.has_colors() else 'blue'
                    
                    # XY 평면 투영
                    ax.scatter(points[:, 0], points[:, 1], c=colors, s=20, alpha=0.7)
            
            ax.set_xlim(-10, 80)
            ax.set_ylim(-15, 15)
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_title(f'Radar Point Cloud Animation - Frame {frame_idx} (t={timestamps[frame_idx]:.1f}s)')
            ax.grid(True, alpha=0.3)
            ax.patch.set_facecolor('black')
            ax.set_aspect('equal')
        
        # 애니메이션 생성
        anim = FuncAnimation(fig, animate, frames=len(point_clouds), 
                           interval=1000//fps, repeat=True)
        
        if save_path:
            print(f"💾 애니메이션을 {save_path}에 저장 중...")
            if save_path.endswith('.gif'):
                writer = PillowWriter(fps=fps)
                anim.save(save_path, writer=writer)
            else:
                anim.save(save_path, fps=fps)
            print("✅ 애니메이션 저장 완료!")
        else:
            plt.show()
        
        return anim

###############################################################################
# 2. Noise Filtering
class StatisticalOutlierRemoval:
    """
    Statistical Outlier Removal filter for point cloud noise filtering using custom KD-Tree.
    
    This class implements the statistical outlier removal algorithm that identifies
    and removes points based on their statistical distribution of distances to neighbors.
    
    Theoretical Background:
    ----------------------
    1. For each point P, find its k nearest neighbors using KD-Tree
    2. Compute the mean distance from P to its k neighbors
    3. Calculate global statistics (mean μ, standard deviation σ) of all mean distances
    4. Set thresholds: lower = μ - (std_ratio × σ), upper = μ + (std_ratio × σ)
    5. Points with mean distances outside [lower, upper] are considered outliers
    
    The assumption is that in a clean point cloud, most points should have
    similar mean distances to their neighbors. Outliers will have significantly
    different (usually larger) mean distances.
    """
    
    def __init__(self, nb_neighbors: int = 20, std_ratio: float = 2.0):
        """
        Initialize the Statistical Outlier Removal filter.
        
        Args:
            nb_neighbors (int): Number of nearest neighbors to consider for each point
                              - Larger values: more robust but slower
                              - Smaller values: faster but less robust
            std_ratio (float): Standard deviation ratio threshold for outlier detection
                             - Larger values: more tolerant (fewer outliers detected)
                             - Smaller values: more strict (more outliers detected)
        """
        self.nb_neighbors = nb_neighbors
        self.std_ratio = std_ratio
        
        # Results from analysis (will be set after filtering)
        self.kdtree_root_ = None
        self.mean_distances_ = None
        self.global_mean_ = None
        self.global_std_ = None
        self.threshold_lower_ = None
        self.threshold_upper_ = None
        self.processing_stats_ = {}
        
    def compute_mean_distances(self, points: np.ndarray) -> np.ndarray:
        """
        Compute the mean distance from each point to its k nearest neighbors using custom KD-Tree.
        
        This method follows the theoretical procedure:
        1. Build a KD-Tree from all points for efficient neighbor search
        2. For each point, find its k nearest neighbors using the KD-Tree
        3. Calculate the mean distance from each point to its neighbors
        4. Return array of mean distances for all points
        
        Args:
            points (np.ndarray): Input point cloud of shape (N, D)
            
        Returns:
            np.ndarray: Mean distances for each point of shape (N,)
        """
        print(f"Computing mean distances for {len(points)} points using custom KD-Tree...")
        print(f"Building KD-Tree structure...")
        
        # Step 1: Build KD-Tree from all points
        kdtree_build_start = time.time()
        # Create point cloud object for KD-Tree
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        self.kdtree_root_ = o3d.geometry.KDTreeFlann(pcd)
        kdtree_build_time = time.time() - kdtree_build_start
        
        if self.kdtree_root_ is None:
            raise ValueError("Failed to build KD-Tree from input points")
        
        print(f"KD-Tree built successfully in {kdtree_build_time:.4f}s. Starting k-NN search for each point...")
        
        # Store processing statistics
        self.processing_stats_['kdtree_build_time'] = kdtree_build_time
        self.processing_stats_['total_points'] = len(points)
        self.processing_stats_['point_dimension'] = points.shape[1]
        
        # Step 2: For each point, find k nearest neighbors and compute mean distance
        mean_distances = np.zeros(len(points))
        knn_search_start = time.time()
        total_searches = 0
        
        for i, query_point in enumerate(points):
            if i % 100 == 0 and i > 0:
                print(f"  Processed {i}/{len(points)} points ({i/len(points)*100:.1f}%)")
            
            # Step 2a: Find k+1 nearest neighbors (including the point itself)
            [_, neighbor_indices, squared_distances] = self.kdtree_root_.search_knn_vector_3d(xxxxxx, xxxxxx) # TODO: k-NN 검색 함수의 인자를 채워주세요.
            total_searches += 1
            
            # Step 2b: Remove the first neighbor (the point itself with distance 0)
            # The first neighbor should be the point itself with very small distance (due to floating point precision)
            if len(neighbor_indices) > 1:
                # Skip the first neighbor (self) and take the next k neighbors
                actual_neighbor_distances = xxxxxx # TODO: squared_distances에서 자신과의 거리를 제외해주세요.
                
                # Convert squared distances to actual distances
                neighbor_distances = np.sqrt(actual_neighbor_distances)
                
                # Step 2c: Compute mean distance to k neighbors
                mean_distances[i] = np.mean(neighbor_distances)
            else:
                # If only one neighbor found (shouldn't happen in normal cases)
                mean_distances[i] = 0.0
                print(f"Warning: Only found {len(neighbor_indices)} neighbor(s) for point {i}")
        
        knn_search_time = time.time() - knn_search_start
        
        print(f"Mean distance computation completed in {knn_search_time:.4f}s.")
        print(f"Average time per k-NN search: {knn_search_time/total_searches*1000:.3f}ms")
        print(f"Distance range: [{np.min(mean_distances):.4f}, {np.max(mean_distances):.4f}]")
        
        # Store more processing statistics
        self.processing_stats_['knn_search_time'] = knn_search_time
        self.processing_stats_['total_knn_searches'] = total_searches
        self.processing_stats_['avg_search_time_ms'] = knn_search_time/total_searches*1000
        
        return mean_distances
    
    def compute_statistics(self, mean_distances: np.ndarray) -> Tuple[float, float, float, float]:
        """
        Compute global statistics of mean distances.
        
        Args:
            mean_distances (np.ndarray): Mean distances for each point
            
        Returns:
            Tuple[float, float, float, float]: global_mean, global_std, threshold_lower, threshold_upper
        """
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        
        threshold_lower = xxxxxx # TODO: self.std_ratio를 사용하여 하한 임계값 계산
        threshold_upper = xxxxxx # TODO: self.std_ratio를 사용하여 상한 임계값 계산
        
        return global_mean, global_std, threshold_lower, threshold_upper
    
    def find_outliers(self, mean_distances: np.ndarray, 
                     threshold_lower: float, threshold_upper: float) -> np.ndarray:
        """
        Find outlier indices based on statistical thresholds.
        
        Args:
            mean_distances (np.ndarray): Mean distances for each point
            threshold_lower (float): Lower threshold for outlier detection
            threshold_upper (float): Upper threshold for outlier detection
            
        Returns:
            np.ndarray: Boolean mask where True indicates outliers
        """
        outlier_mask = (mean_distances < threshold_lower) | (mean_distances > threshold_upper)
        return outlier_mask
    
    def filter_points_sor(self, points: np.ndarray, return_outliers: bool = False) -> Tuple[np.ndarray, np.ndarray]:
        """
        Apply statistical outlier removal to the input point cloud.
        
        Args:
            points (np.ndarray): Input point cloud of shape (N, D)
            return_outliers (bool): If True, also return the outlier points
            
        Returns:
            Tuple[np.ndarray, np.ndarray]: (filtered_points, outlier_indices or outlier_points)
        """
        start_time = time.time()
        
        print(f"Applying Statistical Outlier Removal...")
        print(f"Parameters: nb_neighbors={self.nb_neighbors}, std_ratio={self.std_ratio}")
        print(f"Input points: {len(points)}")
        
        # Step 1: Compute mean distances
        self.mean_distances_ = self.compute_mean_distances(points) # TODO: 커스텀 Statistical Outlier Removal 클래스의 compute_mean_distances 함수 완성
        
        # Step 2: Compute global statistics
        self.global_mean_, self.global_std_, self.threshold_lower_, self.threshold_upper_ = \
            self.compute_statistics(self.mean_distances_) # TODO: 커스텀 Statistical Outlier Removal 클래스의 compute_statistics 함수 완성
        
        print(f"Global mean distance: {self.global_mean_:.4f}")
        print(f"Global std distance: {self.global_std_:.4f}")
        print(f"Threshold range: [{self.threshold_lower_:.4f}, {self.threshold_upper_:.4f}]")
        
        # Step 3: Find outliers
        outlier_mask = self.find_outliers(self.mean_distances_, 
                                         self.threshold_lower_, self.threshold_upper_)
        
        # Step 4: Filter points
        inlier_mask = ~outlier_mask
        filtered_points = xxxxxx # TODO: inlier_mask를 사용하여 필터링된 포인트를 선택
        
        num_outliers = np.sum(outlier_mask)
        num_inliers = len(filtered_points)
        
        print(f"Outliers detected: {num_outliers}")
        print(f"Inliers remaining: {num_inliers}")
        print(f"Outlier ratio: {num_outliers/len(points)*100:.2f}%")
        print(f"Processing time: {time.time() - start_time:.4f} seconds")
        
        if return_outliers:
            outlier_points = points[outlier_mask]
            return filtered_points, outlier_points
        else:
            return filtered_points, np.where(outlier_mask)[0]
    
    def analyze_point_neighborhoods(self, points: np.ndarray, sample_points: List[int] = None) -> Dict[str, Any]:
        """
        Analyze the neighborhood structure of specific points for educational purposes.
        
        Args:
            points (np.ndarray): Original point cloud
            sample_points (List[int]): Indices of points to analyze. If None, analyze random sample.
            
        Returns:
            Dict[str, Any]: Analysis results with neighborhood information
        """
        if self.kdtree_root_ is None:
            raise ValueError("Must run filter_points_sor() first to build KD-Tree")
        
        if sample_points is None:
            # Select 5 random points for analysis
            sample_points = np.random.choice(len(points), size=min(5, len(points)), replace=False)
        
        analysis = {
            'sample_points': sample_points,
            'neighborhood_analysis': []
        }
        
        print("=== Neighborhood Analysis ===")
        for i, point_idx in enumerate(sample_points):
            query_point = points[point_idx]
            
            # Find neighbors
            [_, neighbor_indices, squared_distances] = self.kdtree_root_.search_knn_vector_3d(query_point, self.nb_neighbors + 1)
            
            # Remove self and convert to actual distances
            neighbor_indices = neighbor_indices[1:]
            distances = np.sqrt(squared_distances[1:])
            mean_dist = np.mean(distances)
            
            point_analysis = {
                'point_index': int(point_idx),
                'coordinates': query_point.tolist(),
                'neighbor_indices': [int(idx) for idx in neighbor_indices],
                'distances': distances.tolist(),
                'mean_distance': float(mean_dist),
                'min_distance': float(np.min(distances)),
                'max_distance': float(np.max(distances)),
                'std_distance': float(np.std(distances))
            }
            
            if self.mean_distances_ is not None:
                stored_mean = self.mean_distances_[point_idx]
                point_analysis['stored_mean_distance'] = float(stored_mean)
                point_analysis['is_outlier'] = self._is_point_outlier(stored_mean)
            
            analysis['neighborhood_analysis'].append(point_analysis)
            
            print(f"\nPoint {point_idx} at {query_point}:")
            print(f"  Neighbors: {neighbor_indices[:5]}{'...' if len(neighbor_indices) > 5 else ''}")
            print(f"  Mean distance: {mean_dist:.4f}")
            print(f"  Distance range: [{np.min(distances):.4f}, {np.max(distances):.4f}]")
            if self.mean_distances_ is not None:
                print(f"  Outlier status: {'OUTLIER' if self._is_point_outlier(stored_mean) else 'INLIER'}")
        
        return analysis
    
    def _is_point_outlier(self, mean_distance: float) -> bool:
        """Check if a point is an outlier based on its mean distance."""
        if self.threshold_lower_ is None or self.threshold_upper_ is None:
            return False
        return mean_distance < self.threshold_lower_ or mean_distance > self.threshold_upper_
    
    def get_processing_summary(self) -> Dict[str, Any]:
        """
        Get a summary of the processing statistics for educational analysis.
        
        Returns:
            Dict[str, Any]: Processing statistics and algorithm performance metrics
        """
        if not self.processing_stats_:
            return {"error": "No processing statistics available. Run filter_points_sor() first."}
        
        summary = {
            'algorithm_parameters': {
                'nb_neighbors': self.nb_neighbors,
                'std_ratio': self.std_ratio
            },
            'data_statistics': {
                'total_points': self.processing_stats_.get('total_points', 0),
                'point_dimension': self.processing_stats_.get('point_dimension', 0),
            },
            'performance_metrics': {
                'kdtree_build_time_s': self.processing_stats_.get('kdtree_build_time', 0),
                'knn_search_time_s': self.processing_stats_.get('knn_search_time', 0),
                'total_knn_searches': self.processing_stats_.get('total_knn_searches', 0),
                'avg_search_time_ms': self.processing_stats_.get('avg_search_time_ms', 0),
            },
            'filtering_results': {}
        }
        
        if self.mean_distances_ is not None:
            summary['filtering_results'].update({
                'global_mean_distance': float(self.global_mean_),
                'global_std_distance': float(self.global_std_),
                'threshold_lower': float(self.threshold_lower_),
                'threshold_upper': float(self.threshold_upper_),
                'distance_range': [float(np.min(self.mean_distances_)), float(np.max(self.mean_distances_))]
            })
        
        return summary
    
    def explain_algorithm_steps(self) -> None:
        """
        Print a detailed explanation of the Statistical Outlier Removal algorithm steps.
        """
        print("=" * 60)
        print("Statistical Outlier Removal Algorithm - Step by Step")
        print("=" * 60)
        print(
            """
THEORETICAL BACKGROUND:
The algorithm assumes that in a clean point cloud, most points should have
similar mean distances to their k nearest neighbors. Outlier points will
typically have significantly different (usually larger) mean distances.

ALGORITHM STEPS:

Step 1: KD-Tree Construction
   - Build a KD-Tree from all input points for efficient neighbor search
   - Time complexity: O(n log n) where n is the number of points
   - Space complexity: O(n)

Step 2: Neighborhood Analysis
   For each point P in the point cloud:
   a) Use KD-Tree to find k nearest neighbors of P
   b) Calculate distances from P to each of its k neighbors
   c) Compute mean distance: μ_P = (1/k) * Σ(distance(P, neighbor_i))

Step 3: Statistical Analysis
   - Collect all mean distances: [μ_P1, μ_P2, ..., μ_Pn]
   - Calculate global statistics:
     * Global mean: μ = (1/n) * Σ(μ_Pi)
     * Global standard deviation: σ = sqrt((1/n) * Σ(μ_Pi - μ)²)

Step 4: Threshold Calculation
   - Lower threshold: threshold_lower = μ - (std_ratio * σ)
   - Upper threshold: threshold_upper = μ + (std_ratio * σ)

Step 5: Outlier Classification
   A point P is classified as an outlier if:
   μ_P < threshold_lower OR μ_P > threshold_upper

PARAMETERS:
- nb_neighbors (k): Number of neighbors to consider
  * Larger k: More robust, slower computation
  * Smaller k: Faster, but may be less reliable
  
- std_ratio: Controls sensitivity of outlier detection
  * Larger std_ratio: More tolerant (fewer outliers detected)
  * Smaller std_ratio: More strict (more outliers detected)
  * Typical values: 1.0 (strict) to 3.0 (tolerant)

COMPUTATIONAL COMPLEXITY:
- KD-Tree construction: O(n log n)
- k-NN search for all points: O(n log n) average case
- Overall: O(n log n)
            """
        )
        print("=" * 60)


def statistical_outlier_removal(points: np.ndarray, 
                               nb_neighbors: int = 20, 
                               std_ratio: float = 2.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convenience function for statistical outlier removal.
    
    Args:
        points (np.ndarray): Input point cloud of shape (N, D)
        nb_neighbors (int): Number of nearest neighbors to consider
        std_ratio (float): Standard deviation ratio threshold
        
    Returns:
        Tuple[np.ndarray, np.ndarray]: (filtered_points, outlier_indices)
    """
    filter_obj = StatisticalOutlierRemoval(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return filter_obj.filter_points_sor(points)


def visualize_outlier_removal_2d(original_points: np.ndarray, 
                                filtered_points: np.ndarray, 
                                outlier_points: np.ndarray,
                                title: str = "Statistical Outlier Removal Results") -> None:
    """
    Visualize the results of statistical outlier removal for 2D point clouds.
    
    Args:
        original_points (np.ndarray): Original point cloud
        filtered_points (np.ndarray): Points after filtering
        outlier_points (np.ndarray): Detected outlier points
        title (str): Plot title
    """
    plt.figure(figsize=(15, 5))
    
    # Original points
    plt.subplot(1, 3, 1)
    plt.scatter(original_points[:, 0], original_points[:, 1], c='gray', s=20, alpha=0.6)
    plt.title(f'Original Points\n({len(original_points)} points)')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # Filtered points vs outliers
    plt.subplot(1, 3, 2)
    plt.scatter(filtered_points[:, 0], filtered_points[:, 1], c='blue', s=20, alpha=0.7, label='Inliers')
    if len(outlier_points) > 0:
        plt.scatter(outlier_points[:, 0], outlier_points[:, 1], c='red', s=30, alpha=0.8, label='Outliers')
    plt.title(f'Filtering Results\nInliers: {len(filtered_points)}, Outliers: {len(outlier_points)}')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # Filtered points only
    plt.subplot(1, 3, 3)
    plt.scatter(filtered_points[:, 0], filtered_points[:, 1], c='green', s=20, alpha=0.7)
    plt.title(f'Filtered Points\n({len(filtered_points)} points)')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.suptitle(title)
    plt.tight_layout()
    plt.show()


def visualize_distance_distribution(mean_distances: np.ndarray, 
                                  global_mean: float, 
                                  global_std: float,
                                  threshold_lower: float, 
                                  threshold_upper: float,
                                  outlier_mask: np.ndarray) -> None:
    """
    Visualize the distribution of mean distances and outlier thresholds.
    
    Args:
        mean_distances (np.ndarray): Mean distances for each point
        global_mean (float): Global mean distance
        global_std (float): Global standard deviation
        threshold_lower (float): Lower threshold
        threshold_upper (float): Upper threshold
        outlier_mask (np.ndarray): Boolean mask for outliers
    """
    plt.figure(figsize=(12, 5))
    
    # Distance distribution histogram
    plt.subplot(1, 2, 1)
    plt.hist(mean_distances, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
    plt.axvline(global_mean, color='blue', linestyle='--', linewidth=2, label=f'Mean: {global_mean:.4f}')
    plt.axvline(threshold_lower, color='red', linestyle='--', linewidth=2, label=f'Lower: {threshold_lower:.4f}')
    plt.axvline(threshold_upper, color='red', linestyle='--', linewidth=2, label=f'Upper: {threshold_upper:.4f}')
    plt.xlabel('Mean Distance to Neighbors')
    plt.ylabel('Frequency')
    plt.title('Distribution of Mean Distances')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Outlier vs inlier distances
    plt.subplot(1, 2, 2)
    inlier_distances = mean_distances[~outlier_mask]
    outlier_distances = mean_distances[outlier_mask]
    
    plt.hist(inlier_distances, bins=30, alpha=0.7, color='green', label=f'Inliers ({len(inlier_distances)})')
    if len(outlier_distances) > 0:
        plt.hist(outlier_distances, bins=20, alpha=0.7, color='red', label=f'Outliers ({len(outlier_distances)})')
    
    plt.xlabel('Mean Distance to Neighbors')
    plt.ylabel('Frequency')
    plt.title('Inliers vs Outliers Distribution')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def create_noisy_point_cloud_2d(n_points: int = 1000, outlier_ratio: float = 0.1) -> np.ndarray:
    """
    Create a 2D point cloud with noise for testing purposes.
    
    Args:
        n_points (int): Total number of points
        outlier_ratio (float): Ratio of noise points (0.0 to 1.0)
        
    Returns:
        np.ndarray: 2D point cloud with noise
    """
    np.random.seed(42)
    
    n_clean = int(n_points * (1 - outlier_ratio))
    n_noise = n_points - n_clean
    
    # Create clean points (clustered)
    center1 = np.random.multivariate_normal([2, 2], [[0.5, 0], [0, 0.5]], n_clean // 2)
    center2 = np.random.multivariate_normal([5, 5], [[0.3, 0], [0, 0.3]], n_clean // 2)
    clean_points = np.vstack([center1, center2])
    
    # Create noise points (random)
    noise_points = np.random.uniform(-1, 8, (n_noise, 2))
    
    # Combine
    all_points = np.vstack([clean_points, noise_points])
    
    # Shuffle
    indices = np.random.permutation(len(all_points))
    return all_points[indices]


# Example usage and testing
if __name__ == "__main__":
    print("=== Custom Statistical Outlier Removal Demo (Using Custom KD-Tree) ===")
    
    # Step 1: Explain the algorithm
    print("\n" + "="*60)
    print("STEP 1: Understanding the Algorithm")
    print("="*60)
    filter_obj = StatisticalOutlierRemoval(nb_neighbors=10, std_ratio=2.0)
    filter_obj.explain_algorithm_steps()
    
    # Step 2: Create test data
    print("\n" + "="*60)
    print("STEP 2: Creating Test Data")
    print("="*60)
    points_2d = create_noisy_point_cloud_2d(n_points=200, outlier_ratio=0.15)
    print(f"Created {len(points_2d)} points with ~15% noise")
    print(f"Point cloud dimensions: {points_2d.shape}")
    
    # Step 3: Apply statistical outlier removal with detailed logging
    print("\n" + "="*60)
    print("STEP 3: Applying Statistical Outlier Removal")
    print("="*60)
    print(f"Algorithm parameters:")
    print(f"  - Number of neighbors (k): {filter_obj.nb_neighbors}")
    print(f"  - Standard deviation ratio: {filter_obj.std_ratio}")
    print()
    
    filtered_points, outlier_points = filter_obj.filter_points_sor(points_2d, return_outliers=True)
    
    # Step 4: Get processing summary
    print("\n" + "="*60)
    print("STEP 4: Processing Summary")
    print("="*60)
    summary = filter_obj.get_processing_summary()
    
    print("Algorithm Performance:")
    perf = summary['performance_metrics']
    print(f"  - KD-Tree build time: {perf['kdtree_build_time_s']:.4f}s")
    print(f"  - Total k-NN search time: {perf['knn_search_time_s']:.4f}s")
    print(f"  - Average search time per point: {perf['avg_search_time_ms']:.3f}ms")
    print(f"  - Total k-NN searches performed: {perf['total_knn_searches']}")
    
    if summary['filtering_results']:
        print("\nFiltering Results:")
        results = summary['filtering_results']
        print(f"  - Global mean distance: {results['global_mean_distance']:.4f}")
        print(f"  - Global std distance: {results['global_std_distance']:.4f}")
        print(f"  - Threshold range: [{results['threshold_lower']:.4f}, {results['threshold_upper']:.4f}]")
        print(f"  - Distance range: [{results['distance_range'][0]:.4f}, {results['distance_range'][1]:.4f}]")
    
    # Step 5: Analyze specific point neighborhoods
    print("\n" + "="*60)
    print("STEP 5: Neighborhood Analysis")
    print("="*60)
    
    # Choose specific points for analysis (some inliers and potential outliers)
    outlier_indices = np.where(filter_obj.find_outliers(filter_obj.mean_distances_, 
                                                       filter_obj.threshold_lower_, 
                                                       filter_obj.threshold_upper_))[0]
    inlier_indices = np.where(~filter_obj.find_outliers(filter_obj.mean_distances_, 
                                                       filter_obj.threshold_lower_, 
                                                       filter_obj.threshold_upper_))[0]
    
    # Select 3 inliers and 2 outliers for detailed analysis
    sample_points = []
    if len(inlier_indices) >= 3:
        sample_points.extend(np.random.choice(inlier_indices, 3, replace=False))
    if len(outlier_indices) >= 2:
        sample_points.extend(np.random.choice(outlier_indices, 2, replace=False))
    
    analysis = filter_obj.analyze_point_neighborhoods(points_2d, sample_points)
    
    # Step 6: Visualize results
    print("\n" + "="*60)
    print("STEP 6: Visualization")
    print("="*60)
    print("Generating visualizations...")
    
    visualize_outlier_removal_2d(points_2d, filtered_points, outlier_points, 
                                title="Statistical Outlier Removal using Custom KD-Tree")
    
    # Visualize distance distribution
    outlier_mask = filter_obj.find_outliers(filter_obj.mean_distances_, 
                                           filter_obj.threshold_lower_, 
                                           filter_obj.threshold_upper_)
    
    visualize_distance_distribution(filter_obj.mean_distances_, 
                                  filter_obj.global_mean_, 
                                  filter_obj.global_std_,
                                  filter_obj.threshold_lower_, 
                                  filter_obj.threshold_upper_,
                                  outlier_mask)
    
    # Step 7: Summary
    print("\n" + "="*60)
    print("STEP 7: Final Summary")
    print("="*60)
    print(f"Original points: {len(points_2d)}")
    print(f"Filtered points: {len(filtered_points)}")
    print(f"Outliers removed: {len(outlier_points)}")
    print(f"Outlier ratio: {len(outlier_points)/len(points_2d)*100:.2f}%")
    print(f"Data quality improvement: {len(filtered_points)/len(points_2d)*100:.2f}% of data retained")
    
    print("\nKey insights:")
    print("- Custom KD-Tree provides efficient neighbor search")
    print("- Statistical analysis identifies points with unusual neighborhood distances")
    print("- Threshold-based classification separates clean data from noise")
    print("- Educational implementation shows each algorithmic step clearly")
    
    print("\n=== Demo completed successfully! ===")
    print("The custom implementation using KD-Tree demonstrates the theoretical")
    print("principles of Statistical Outlier Removal in an educational context.")

###############################################################################
# 3. Translation
def custom_translation(pcd: o3d.geometry.PointCloud, vector: np.ndarray) -> o3d.geometry.PointCloud:
    """translate xyz coordinates.

    Args:
        pcd: PointCloud object
        vector: (3, 1)

    Return:
        pcd_translation: PointCloud object
    """
    xyz = np.asarray(pcd.points)
    translated_xyz = xxxxxx # TODO: np.newaxis를 사용하여 vector를 xyz에 더하기

    pcd_translation = o3d.geometry.PointCloud()
    pcd_translation.points = o3d.utility.Vector3dVector(translated_xyz)

    return pcd_translation

###############################################################################
# 4. Rotation
def custom_get_rotation_matrix(angle: np.ndarray) -> np.ndarray:
    """Get rotation matrix from angle.
    
    Args:
        angle: [roll, pitch, yaw] (degree)
    Return:
        rotation_matrix: (3, 3)
    """
    roll = np.deg2rad(angle[0])
    pitch = np.deg2rad(angle[1])
    yaw = np.deg2rad(angle[2])
    
    rotation_matrix_roll = np.array(
        [
            xxxxxx
        ]
    ) # TODO: roll 회전 행렬 구현
    rotation_matrix_pitch = np.array(
        [
            xxxxxx
        ]
    ) # TODO: pitch 회전 행렬 구현
    rotation_matrix_yaw = np.array(
        [
            xxxxxx
        ]
    ) # TODO: yaw 회전 행렬 구현
    
    rotation_matrix = xxxxxx # TODO: roll, pitch, yaw 회전 행렬을 곱하여 최종 회전 행렬 구현 (행렬 곱: @ 연산자 사용)
    
    return rotation_matrix

def custom_rotation(pcd: o3d.geometry.PointCloud, rotation_matrix: np.ndarray, center: np.ndarray = np.array([0, 0, 0])) -> o3d.geometry.PointCloud:
    """Rotate xyz.

    Args:
        pcd: PointCloud object
        rotation_matrix: (3, 3)
        center: (3)
    Return:
        pcd_rotation: PointCloud object
    """
    xyz = np.asarray(pcd.points)
    center = np.asarray(center)

    rotation_xyz = xxxxxx # TODO: 회전 행렬과 center 좌표를 적용하여 xyz 좌표 회전 (np.asarray(pcd.points).shape 확인 필수)
    rotation_xyz = xxxxxx # TODO: 회전 후 center 좌표를 다시 더하여 xyz 좌표 원위치

    pcd_rotation = o3d.geometry.PointCloud()
    pcd_rotation.points = o3d.utility.Vector3dVector(rotation_xyz)

    return pcd_rotation

###############################################################################
# 5. Transformation matrix
def custom_get_transformation_matrix(translation_vector: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
    """Get transformation matrix from translation vector, rotation matrix.
    
    Args:
        translation_vector: (3)
        rotation_matrix: (3, 3)
    Return:
        transformation_matrix: (4, 4)
    """
    transformation_matrix = np.eye(4)
    transformation_matrix[xxxxxx, xxxxxx] = rotation_matrix    # TODO: rotation_matrix을 transformation_matrix에 삽입
    transformation_matrix[xxxxxx, xxxxxx] = translation_vector # TODO: translation_vector를 transformation_matrix에 삽입

    return transformation_matrix

def custom_pcd_transformation(pcd: o3d.geometry.PointCloud, transformation_matrix: np.ndarray) -> o3d.geometry.PointCloud:
    """Transform xyz.

    Args:
        pcd: PointCloud object
        transformation_matrix: (4, 4)
    Return:
        pcd_transformation: PointCloud object
    """
    xyz = np.asarray(pcd.points)
    homogeneous_xyz = xxxxxx # TODO: (N, 4) 형태로 xyz 좌표에 1을 추가하여 동차 좌표로 변환
    transformation_xyz = xxxxxx # TODO: transformation_matrix와 homogeneous_xyz를 곱하여 변환된 좌표 계산 (np.matmul 사용)
    transformation_xyz = transformation_xyz[:, :3] # 포인트 클라우드를 다시 (N, 3) 형태로 변환

    pcd_transformation = o3d.geometry.PointCloud()
    pcd_transformation.points = o3d.utility.Vector3dVector(transformation_xyz)

    return pcd_transformation


###############################################################################
# 6. DBSCAN 클러스터링 구현
def custom_dbscan_clustering(pcd: o3d.geometry.PointCloud, 
                            epsilon: float, 
                            min_points: int) -> Tuple[np.ndarray, int]:
    """DBSCAN 클러스터링 수행
    
    Args:
        pcd: 포인트 클라우드
        epsilon: 이웃 반지름
        min_points: 코어 포인트가 되기 위한 최소 이웃 개수
    
    Returns:
        labels: 클러스터 라벨들 (-1은 노이즈)
        n_clusters: 발견된 클러스터 개수
    """
    points = np.array(pcd.points)
    n_points = len(points)

    # KD-Tree 미리 구축 (한 번만)
    kdtree = o3d.geometry.KDTreeFlann(pcd)
    
    # 라벨 초기화 (-1: 미분류, 0~: 클러스터 ID)
    labels = xxxxxx # TODO: np.full을 사용하여 -1로 초기화된 라벨 배열 생성
    cluster_id = 0
    
    for point_idx in range(n_points):
        # 이미 분류된 포인트는 건너뛰기
        if labels[point_idx] != -1:
            continue
        
        # 이웃 포인트들 찾기
        [num_neighbors, neighbors, distances] = xxxxxx # TODO: kdtree의 search_radius_vector_3d 함수 및 현재 포인트 위치와 epsilon을 사용하여 이웃 포인트 찾기
        
        # 코어 포인트가 아닌 경우 (이웃이 min_points보다 적음)
        if xxxxxx < min_points: # TODO: 이웃 개수가 min_points보다 적은지 확인
            # 노이즈로 분류 (라벨 -1 유지)
            continue
        
        # 새로운 클러스터 시작
        labels[point_idx] = xxxxxx # TODO: 현재 클러스터 ID로 라벨 업데이트
        
        # 이웃들을 클러스터에 추가 (너비 우선 탐색)
        seed_set = list(neighbors)
        i = 0
        while i < len(seed_set):
            current_point = xxxxxx # TODO: 현재 처리 중인 포인트
            
            # 노이즈였던 포인트를 클러스터에 추가
            if labels[current_point] == -1:
                labels[current_point] = xxxxxx # TODO: 현재 클러스터 ID로 라벨 업데이트
                
                # 현재 포인트도 코어 포인트인지 확인
                [num_neighbors, current_neighbors, distances] = xxxxxx # TODO: kdtree의 search_radius_vector_3d 함수 및 현재 포인트 위치와 epsilon을 사용하여 이웃 포인트 찾기
                if xxxxxx >= min_points: # TODO: 이웃 개수가 min_points 이상인지 확인
                    # 새로운 이웃들을 seed_set에 추가
                    for neighbor in current_neighbors:
                        if neighbor not in seed_set:
                            seed_set.append(xxxxxx) # TODO: seed_set에 새로운 이웃 추가
            
            i += 1
        
        cluster_id += 1
    
    return labels, cluster_id

# 클러스터링 성능 평가
def custom_clustering_metrics(true_labels: np.ndarray, 
                             pred_labels: np.ndarray) -> Dict[str, float]:
    """클러스터링 성능 평가 지표들 계산
    
    Args:
        true_labels: 실제 라벨들
        pred_labels: 예측 라벨들
    
    Returns:
        metrics: 평가 지표들 딕셔너리
    """
    from collections import Counter
    
    # 라벨을 0부터 시작하도록 정규화
    def normalize_labels(labels):
        unique_labels = sorted(set(labels))
        label_map = {old: new for new, old in enumerate(unique_labels)}
        return np.array([label_map[label] for label in labels])
    
    true_normalized = normalize_labels(true_labels)
    pred_normalized = normalize_labels(pred_labels)
    
    n_points = len(true_labels)
    
    # 정확도 계산 (라벨 매칭 고려)
    # 가장 많이 겹치는 라벨 쌍을 찾아서 매칭
    true_clusters = set(true_normalized)
    pred_clusters = set(pred_normalized)
    
    best_accuracy = 0.0
    
    # 모든 가능한 라벨 매핑을 시도 (작은 클러스터 수에서만 가능)
    if len(pred_clusters) <= 10:
        import itertools
        for perm in itertools.permutations(pred_clusters):
            if len(perm) != len(true_clusters):
                continue
            
            mapped_pred = pred_normalized.copy()
            for i, pred_label in enumerate(perm):
                mapped_pred[pred_normalized == pred_label] = i
            
            accuracy = np.mean(true_normalized == mapped_pred)
            best_accuracy = max(best_accuracy, accuracy)
    
    # 클러스터 순도 (Purity) 계산
    total_correct = 0
    for pred_cluster in pred_clusters:
        cluster_mask = (pred_normalized == pred_cluster)
        if np.sum(cluster_mask) > 0:
            cluster_true_labels = true_normalized[cluster_mask]
            most_common_true_label = Counter(cluster_true_labels).most_common(1)[0][0]
            correct_in_cluster = np.sum(cluster_true_labels == most_common_true_label)
            total_correct += correct_in_cluster
    
    purity = total_correct / n_points
    
    return {
        'accuracy': best_accuracy,
        'purity': purity,
        'n_predicted_clusters': len(pred_clusters),
        'n_true_clusters': len(true_clusters)
    }

# Visualization
def visualize_clustering_result(pcd: o3d.geometry.PointCloud, 
                               labels: np.ndarray,
                               centroids: Optional[np.ndarray] = None,
                               title: str = "Clustering Result",
                               show_centroids: bool = True) -> None:
    """클러스터링 결과 시각화
    
    Args:
        pcd: 포인트 클라우드
        labels: 클러스터 라벨들
        centroids: 클러스터 중심점들 (선택사항)
        title: 창 제목
        show_centroids: 중심점 표시 여부
    """
    geometries = []
    
    # 포인트 클라우드 색상 설정
    pcd_copy = pcd.__copy__()
    points = np.array(pcd_copy.points)
    colors = np.zeros((len(points), 3))
    
    # 미리 정의된 색상 팔레트
    color_palette = [
        [1, 0, 0],      # 빨강
        [0, 1, 0],      # 초록
        [0, 0, 1],      # 파랑
        [1, 1, 0],      # 노랑
        [1, 0, 1],      # 마젠타
        [0, 1, 1],      # 시안
        [1, 0.5, 0],    # 주황
        [0.5, 0, 1],    # 보라
        [1, 0.5, 0.5],  # 분홍
        [0.5, 1, 0.5],  # 연초록
    ]
    
    unique_labels = np.unique(labels)
    
    for i, label in enumerate(unique_labels):
        mask = (labels == label)
        if label == -1:
            # 노이즈는 회색으로 표시
            colors[mask] = [0.5, 0.5, 0.5]
        else:
            # 클러스터는 서로 다른 색상으로 표시
            color_idx = label % len(color_palette)
            colors[mask] = color_palette[color_idx]
    
    pcd_copy.colors = o3d.utility.Vector3dVector(colors)
    geometries.append(pcd_copy)
    
    # 중심점 시각화
    if centroids is not None and show_centroids:
        for i, centroid in enumerate(centroids):
            # 중심점을 구로 표시
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
            sphere.translate(centroid)
            
            # 중심점 색상 설정
            color_idx = i % len(color_palette)
            sphere.paint_uniform_color(color_palette[color_idx])
            sphere.compute_vertex_normals()
            geometries.append(sphere)
    
    # 좌표계 추가
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    geometries.append(coordinate_frame)
    
    o3d.visualization.draw_geometries(geometries, window_name=title)