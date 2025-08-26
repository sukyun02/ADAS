"""Point Cloud I/O - Create Point Cloud"""
"""
 * @file        create_pcd.py
 * @brief       Custom point cloud creation algorithms for point cloud processing
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)          
 *
 * @date        2025-07-28 Released by AI Lab, Hanyang University
 * 
"""

import numpy as np
import open3d as o3d
from typing import List, Optional, Tuple

###############################################################################
# 포인트 클라우드 생성 (직선, 구) with noise and outliers
def create_line_pcd(line_params: np.ndarray, num_points: int = 500, noise_std: float = 0.3, outlier_ratio: float = 0.0, file_name: str = None) -> o3d.geometry.PointCloud:
    """노이즈와 아웃라이어가 포함된 직선 포인트 클라우드 생성
    
    직선 방정식: y = ax + b (z = 0으로 고정)
    
    Args:
        line_params: 직선 파라미터 [a, b] (y = ax + b)
        num_points: 전체 포인트 개수
        noise_std: 가우시안 노이즈 표준편차
        outlier_ratio: 아웃라이어 비율 (0.0 ~ 1.0)
        file_name: 저장할 파일 이름 (선택사항)
    
    Returns:
        line_pcd: 생성된 포인트 클라우드
    """
    a, b = line_params
    
    # 인라이어 포인트 개수 계산
    num_inliers = int(num_points * (1 - outlier_ratio))
    num_outliers = num_points - num_inliers
    
    # 1. 인라이어 포인트 생성
    # x 좌표를 -5 ~ 5 범위에서 랜덤 생성
    x_coords = np.random.uniform(-5, 5, num_inliers)
    
    # 직선 방정식에 따라 y 좌표 계산
    y_coords = a * x_coords + b
    
    # z 좌표는 0으로 고정
    z_coords = np.zeros(num_inliers)
    
    # 가우시안 노이즈 추가
    noise = np.random.normal(0, noise_std, (num_inliers, 3))
    inlier_points = np.column_stack([x_coords, y_coords, z_coords]) + noise
    
    # 2. 아웃라이어 포인트 생성 (완전히 랜덤한 위치)
    outlier_points = np.random.uniform(-10, 10, (num_outliers, 3))
    
    # 3. 모든 포인트 결합
    all_points = np.vstack([inlier_points, outlier_points])
    
    # 4. 포인트 클라우드 객체 생성
    line_pcd = o3d.geometry.PointCloud()
    line_pcd.points = o3d.utility.Vector3dVector(all_points)
    
    # 5. 파일 저장 (선택사항)
    if file_name:
        o3d.io.write_point_cloud(file_name, line_pcd)
        print(f"직선 포인트 클라우드가 {file_name}에 저장되었습니다.")
    
    return line_pcd

def create_sphere_pcd(sphere_center: np.ndarray, radius: float, num_points: int = 500, noise_std: float = 0.3, outlier_ratio: float = 0.0, file_name: str = None) -> o3d.geometry.PointCloud:
    """노이즈와 아웃라이어가 포함된 구 포인트 클라우드 생성
    
    구 방정식: (x-cx)² + (y-cy)² + (z-cz)² = r²
    
    Args:
        sphere_center: 구의 중심점 [cx, cy, cz]
        radius: 구의 반지름
        num_points: 전체 포인트 개수
        noise_std: 가우시안 노이즈 표준편차
        outlier_ratio: 아웃라이어 비율 (0.0 ~ 1.0)
        file_name: 저장할 파일 이름 (선택사항)
    
    Returns:
        sphere_pcd: 생성된 포인트 클라우드
    """
    cx, cy, cz = sphere_center
    
    # 인라이어 포인트 개수 계산
    num_inliers = int(num_points * (1 - outlier_ratio))
    num_outliers = num_points - num_inliers
    
    # 1. 인라이어 포인트 생성 (구 표면에 균등 분포)
    # 구면 좌표계를 사용하여 균등 분포 생성
    phi = np.random.uniform(0, 2*np.pi, num_inliers)  # 방위각
    cos_theta = np.random.uniform(-1, 1, num_inliers)  # cos(극각)
    theta = np.arccos(cos_theta)  # 극각
    
    # 구면 좌표를 직교 좌표로 변환
    x_coords = radius * np.sin(theta) * np.cos(phi) + cx
    y_coords = radius * np.sin(theta) * np.sin(phi) + cy
    z_coords = radius * np.cos(theta) + cz
    
    # 가우시안 노이즈 추가
    noise = np.random.normal(0, noise_std, (num_inliers, 3))
    inlier_points = np.column_stack([x_coords, y_coords, z_coords]) + noise
    
    # 2. 아웃라이어 포인트 생성 (완전히 랜덤한 위치)
    bound = radius + 5  # 구 주변 영역에서 아웃라이어 생성
    outlier_points = np.random.uniform(-bound, bound, (num_outliers, 3)) + sphere_center
    
    # 3. 모든 포인트 결합
    all_points = np.vstack([inlier_points, outlier_points])
    
    # 4. 포인트 클라우드 객체 생성
    sphere_pcd = o3d.geometry.PointCloud()
    sphere_pcd.points = o3d.utility.Vector3dVector(all_points)
    
    # 5. 파일 저장 (선택사항)
    if file_name:
        o3d.io.write_point_cloud(file_name, sphere_pcd)
        print(f"구 포인트 클라우드가 {file_name}에 저장되었습니다.")
    
    return sphere_pcd

###############################################################################
# 클러스터링용 포인트 클라우드 생성
def create_clustering_pcd(cluster_centers: List[np.ndarray], 
                         cluster_sizes: List[int],
                         cluster_stds: Optional[List[float]] = None,
                         clutter_points: int = 0,
                         file_name: Optional[str] = None) -> Tuple[o3d.geometry.PointCloud, np.ndarray]:
    """클러스터링 실습을 위한 포인트 클라우드 생성
    
    Args:
        cluster_centers: 각 클러스터의 중심점들 [[x1,y1,z1], [x2,y2,z2], ...]
        cluster_sizes: 각 클러스터의 포인트 개수들
        cluster_stds: 각 클러스터의 표준편차들 (기본값: 모두 1.0)
        clutter_points: 클러터 포인트 개수
        file_name: 저장할 파일 이름 (선택사항)
    
    Returns:
        pcd: 생성된 포인트 클라우드
        true_labels: 실제 클러스터 라벨 (-1은 클러터)
    """
    if cluster_stds is None:
        cluster_stds = [1.0] * len(cluster_centers)
    
    all_points = []
    true_labels = []
    
    # 각 클러스터 포인트 생성
    for i, (center, size, std) in enumerate(zip(cluster_centers, cluster_sizes, cluster_stds)):
        # 가우시안 분포로 클러스터 포인트 생성
        cluster_points = np.random.normal(center, std, (size, 3))
        all_points.append(cluster_points)
        true_labels.extend([i] * size)
    
    # 클러터 포인트 생성
    if clutter_points > 0:
        # 클러스터들의 범위를 계산하여 클러터 영역 설정
        all_cluster_points = np.vstack(all_points)
        min_coords = np.min(all_cluster_points, axis=0) - 3
        max_coords = np.max(all_cluster_points, axis=0) + 3
        
        clutter_pts = np.random.uniform(min_coords, max_coords, (clutter_points, 3))
        all_points.append(clutter_pts)
        true_labels.extend([-1] * clutter_points)  # 클러터는 -1 라벨
    
    # 모든 포인트 결합
    all_points_array = np.vstack(all_points)
    true_labels_array = np.array(true_labels)
    
    # 포인트와 라벨을 함께 섞기
    indices = np.arange(len(all_points_array))
    np.random.shuffle(indices)
    all_points_array = all_points_array[indices]
    true_labels_array = true_labels_array[indices]
    
    # 포인트 클라우드 객체 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points_array)
    
    # 파일 저장 (선택사항)
    if file_name:
        o3d.io.write_point_cloud(file_name, pcd)
        print(f"클러스터링 포인트 클라우드가 {file_name}에 저장되었습니다.")
    
    return pcd, true_labels_array