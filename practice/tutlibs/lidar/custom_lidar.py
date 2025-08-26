"""LiDAR Practice"""
"""
 * @file        custom_lidar.py
 * @brief       Custom LiDAR practice algorithms for point cloud processing
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)          
 *
 * @date        2025-08-12 Released by AI Lab, Hanyang University
 * 
"""

import numpy as np
import open3d as o3d
from typing import Tuple, List, Optional
from tutlibs.pointcloud_io.auxiliary_visualization import get_text_mesh

###############################################################################
# 1. ROI 영역 추출
def custom_extract_roi(pcd: o3d.geometry.PointCloud, 
                      x_range: Tuple[float, float] = (-20, 20),
                      y_range: Tuple[float, float] = (-20, 20),
                      z_range: Tuple[float, float] = (-3, 3)) -> Tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud]:
    """ROI(Region of Interest) 영역 추출
    
    Args:
        pcd: 입력 포인트 클라우드
        x_range: X축 범위 (min, max)
        y_range: Y축 범위 (min, max)
        z_range: Z축 범위 (min, max)
    
    Returns:
        roi_pcd: ROI 영역 내의 포인트 클라우드
        roi_outliers_pcd: ROI 영역 외의 포인트 클라우드
    """
    # 포인트 클라우드를 numpy 배열로 변환
    points = np.array(pcd.points)
    
    # ROI 영역 내의 포인트들만 필터링
    x_mask = (points[:, 0] >= xxxxxx) & (points[:, 0] <= xxxxxx)  # X축 조건 - TODO: x_range 값을 사용하여 필터링
    y_mask = (points[:, 1] >= xxxxxx) & (points[:, 1] <= xxxxxx)  # Y축 조건 - TODO: y_range 값을 사용하여 필터링
    z_mask = (points[:, 2] >= xxxxxx) & (points[:, 2] <= xxxxxx)  # Z축 조건 - TODO: z_range 값을 사용하여 필터링
    
    # 모든 조건을 만족하는 포인트들의 인덱스
    roi_mask = xxxxxx # TODO: 각 mask 영역을 AND 연산하여 roi_mask 생성
    roi_points = points[xxxxxx] # TODO: ROI 영역 내의 포인트들 추출
    roi_outliers = points[xxxxxx] # TODO: ROI 영역 외의 포인트들 추출
    
    # 새로운 포인트 클라우드 생성
    roi_pcd = o3d.geometry.PointCloud()
    roi_pcd.points = o3d.utility.Vector3dVector(roi_points)
    
    # 색상 정보가 있다면 함께 복사
    if len(pcd.colors) > 0:
        colors = np.array(pcd.colors)
        roi_colors = colors[roi_mask]
        roi_pcd.colors = o3d.utility.Vector3dVector(roi_colors)
    
    # 새로운 포인트 클라우드 생성
    roi_outliers_pcd = o3d.geometry.PointCloud()
    roi_outliers_pcd.points = o3d.utility.Vector3dVector(roi_outliers)
    
    # 색상 정보가 있다면 함께 복사
    if len(pcd.colors) > 0:
        colors = np.array(pcd.colors)
        roi_outliers_colors = colors[~roi_mask]
        roi_outliers_pcd.colors = o3d.utility.Vector3dVector(roi_outliers_colors)
    
    return roi_pcd, roi_outliers_pcd

###############################################################################
# 2. k-NN 검색
def custom_brute_force_search(pcd: o3d.geometry.PointCloud, query: np.ndarray, k: int) -> Tuple[np.ndarray, np.ndarray]:
    """Brute-force k-NN 검색
    
    Args:
        pcd: 입력 포인트 클라우드
        query: 쿼리 포인트
        k: 최근접 이웃 수
    
    Returns:
        indices: 최근접 이웃 인덱스
        distances: 최근접 이웃 거리
    """
    # 포인트 클라우드를 numpy 배열로 변환
    points = np.array(pcd.points)
    
    # 쿼리 포인트를 numpy 배열로 변환
    query = np.array(query)
    
    # TODO: 모든 포인트와의 거리 계산
    distances = xxxxxx
    
    # TODO: 거리에 따라 정렬
    sorted_indices = xxxxxx
    
    # TODO: 가장 가까운 k개의 포인트 선택
    k_indices = xxxxxx
    k_distances = xxxxxx
    
    return k_indices, k_distances

###############################################################################
# 3. Least Squares Fitting (최소 자승법)
def custom_least_squares_line_fitting(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    """최소 자승법을 사용한 직선 피팅
    
    직선 방정식: y = ax + b
    목적 함수: minimize Σ(yi - axi - b)²
    
    Args:
        pcd: 포인트 클라우드
    
    Returns:
        line_params: 직선 파라미터 [a, b]
    """
    # 포인트 클라우드를 numpy 배열로 변환
    points = np.array(pcd.points)
    x, y = points[:, 0], points[:, 1]  # z는 사용하지 않음

    # 정규 방정식을 위한 행렬 H 와 관측값 b 구성
    # H * params = b 형태로 만들기
    # H = [x1 1,
    #      x2 1,
    #      ...,
    #      xn 1]
    # b = [y1,
    #      y2,
    #      ...,
    #      yn]
    # params = [a, b]^T

    # 필요한 합들 계산
    n = len(points)

    # 행렬 H 구성
    H = np.column_stack([x, np.ones(n)]) # H 행렬 완성

    # 벡터 b 구성
    b = y # TODO: b 벡터 완성

    # 정규 방정식: H^T * H * params = H^T * b
    HtH = xxxxxx          # (2,2) - TODO: H^T H 행렬 완성
    Ht_b = xxxxxx         # (2,)  - TODO: H^T b 벡터 완성

    # 직접 역행렬을 써서 해 구하기: x_hat = (H^T H)^{-1} H^T b
    try:
        inv_HtH = np.linalg.inv(HtH)
    except np.linalg.LinAlgError:
        # 특이행렬일 때는 의사역행렬 사용
        inv_HtH = np.linalg.pinv(HtH)
    line_params = xxxxxx  # (2,) - TODO: inv_HtH와 Ht_b를 사용하여 line_params 완성

    return line_params

def custom_least_squares_sphere_fitting(pcd: o3d.geometry.PointCloud) -> Tuple[np.ndarray, float]:
    """최소 자승법을 사용한 구 피팅
    
    구 방정식: (x-cx)² + (y-cy)² + (z-cz)² = r²
    확장된 형태: x² + y² + z² - 2cx*x - 2cy*y - 2cz*z + (cx² + cy² + cz² - r²) = 0
    선형화: x² + y² + z² = 2cx*x + 2cy*y + 2cz*z + d (d = r² - cx² - cy² - cz²)
    
    Args:
        pcd: 포인트 클라우드
    
    Returns:
        center: 구의 중심점 [cx, cy, cz]
        radius: 구의 반지름
    """
    # 포인트 클라우드를 numpy 배열로 변환
    points = np.array(pcd.points)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    n = len(points)
    
    # 정규 방정식을 위한 행렬 H 와 관측값 b 구성
    # H * [2cx, 2cy, 2cz, d]^T = b
    H = np.column_stack([x, y, z, np.ones(n)])  # TODO: H 행렬 완성
    b = x ** 2 + y ** 2 + z ** 2 # TODO: b 벡터 완성
    
    # 정규 방정식: H^T * H * params = H^T * b
    HtH = xxxxxx          # (4,4) - TODO: H^T H 행렬 완성
    Ht_b = xxxxxx         # (4,)  - TODO: H^T b 벡터 완성
    
    # 직접 역행렬을 써서 해 구하기: x_hat = (H^T H)^{-1} H^T b
    try:
        inv_HtH = np.linalg.inv(HtH)
    except np.linalg.LinAlgError:
        # 특이행렬일 때는 의사역행렬 사용
        inv_HtH = np.linalg.pinv(HtH)
    params = xxxxxx  # (4,) - TODO: inv_HtH와 Ht_b를 사용하여 line_params 완성
    
    # 파라미터 추출
    cx, cy, cz = xxxxxx, xxxxxx, xxxxxx # TODO: params에서 cx, cy, cz 추출
    d = xxxxxx # TODO: params에서 d 추출
    
    # 반지름 계산: r² = cx² + cy² + cz² + d
    radius_squared = cx**2 + cy**2 + cz**2 + d  # TODO: cx, cy, cz, d를 사용하여 radius_squared 계산
    radius0 = np.sqrt(max(0, radius_squared))  # 음수 방지
    center0 = np.array([cx, cy, cz], dtype=float)
    
    return center0, radius0

def custom_refine_sphere_gauss_newton(
    pcd: o3d.geometry.PointCloud,
    center: np.ndarray,
    radius: float,
    max_iter: int = 10,
    tol: float = 1e-6
) -> Tuple[np.ndarray, float]:
    """
    Gauss–Newton으로 residual = dist(point, sphere) - radius 를 최소화
    """
    points = np.array(pcd.points)
    
    for _ in range(max_iter):
        diffs = points - center                 # (N,3)
        dists = np.linalg.norm(diffs, axis=1)   # (N,)
        # zero-division 방지
        dists = np.where(dists == 0, 1e-8, dists)

        # residual 벡터: r_i = dist_i - radius
        resid = dists - radius                  # (N,)

        # Jacobian J (N×4): ∂r_i/∂[cx,cy,cz,r] 
        J = np.empty((points.shape[0], 4))
        J[:, 0] = (center[0] - points[:, 0]) / dists
        J[:, 1] = (center[1] - points[:, 1]) / dists
        J[:, 2] = (center[2] - points[:, 2]) / dists
        J[:, 3] = -1.0

        # 정상방정식 (J^T J) Δ = - J^T resid
        JTJ = J.T @ J             # (4,4)
        JTr = J.T @ resid         # (4,)
        try:
            inv_JTJ = np.linalg.inv(JTJ)
        except np.linalg.LinAlgError:
            inv_JTJ = np.linalg.pinv(JTJ)
        delta = inv_JTJ @ (-JTr)

        # 파라미터 업데이트
        center += delta[:3]
        radius += delta[3]

        # 수렴판정
        if np.linalg.norm(delta) < tol:
            break

    return center, radius

def custom_refine_sphere_gauss_newton_one_step(
    pcd: o3d.geometry.PointCloud,
    center: np.ndarray,
    radius: float,
) -> Tuple[np.ndarray, float, np.ndarray]:
    """
    Gauss–Newton으로 residual = dist(point, sphere) - radius 를 최소화
    """
    points = np.array(pcd.points)
    
    diffs = points - center                 # (N,3)
    dists = np.linalg.norm(diffs, axis=1)   # (N,)
    # zero-division 방지
    dists = np.where(dists == 0, 1e-8, dists)

    # residual 벡터: r_i = dist_i - radius
    resid = dists - radius                  # (N,)

    # Jacobian J (N×4): ∂r_i/∂[cx,cy,cz,r] 
    J = np.empty((points.shape[0], 4))
    J[:, 0] = (center[0] - points[:, 0]) / dists
    J[:, 1] = (center[1] - points[:, 1]) / dists
    J[:, 2] = (center[2] - points[:, 2]) / dists
    J[:, 3] = -1.0

    # 정상방정식 (J^T J) Δ = - J^T resid
    JTJ = J.T @ J             # (4,4)
    JTr = J.T @ resid         # (4,)
    try:
        inv_JTJ = np.linalg.inv(JTJ)
    except np.linalg.LinAlgError:
        inv_JTJ = np.linalg.pinv(JTJ)
    delta = inv_JTJ @ (-JTr)

    # 파라미터 업데이트
    center += delta[:3]
    radius += delta[3]

    return center, radius, delta

###############################################################################
# 4. RANSAC 알고리즘 단계별 구현
def custom_ransac_sample_points(pcd: o3d.geometry.PointCloud, n_samples: int) -> Tuple[np.ndarray, np.ndarray]:
    """RANSAC을 위한 랜덤 포인트 샘플링
    
    Args:
        pcd: 포인트 클라우드
        n_samples: 샘플링할 포인트 개수
    
    Returns:
        sampled_points: 샘플링된 포인트들
        sampled_indices: 샘플링된 포인트들의 인덱스
    """
    points = np.array(pcd.points)
    n_points = len(points)
    
    # 중복 없이 랜덤 인덱스 선택
    sampled_indices = xxxxxx # TODO: np.random.choice를 사용하여 중복 없이(replace=False) 랜덤 인덱스 선택
    sampled_points = points[xxxxxx] # TODO: sampled_indices를 사용하여 샘플링된 포인트들 선택
    
    return sampled_points, sampled_indices

def custom_fit_line_from_samples(sampled_points: np.ndarray) -> np.ndarray:
    """샘플링된 포인트들로부터 직선 피팅
    
    Args:
        sampled_points: 샘플링된 포인트들 (최소 2개)
    
    Returns:
        line_params: 직선 파라미터 [a, b]
    """
    if len(sampled_points) < 2:
        raise ValueError("직선 피팅을 위해서는 최소 2개의 포인트가 필요합니다.")
    
    x, y = sampled_points[:, 0], sampled_points[:, 1]  # z는 사용하지 않음
    
    # 최소 자승법과 동일한 방식으로 피팅
    n = len(sampled_points)

    # 행렬 H 구성
    H = np.column_stack([x, np.ones(n)]) # TODO: H 행렬 완성

    # 벡터 b 구성
    b = y # TODO: b 벡터 완성

    # 정규 방정식: H^T * H * params = H^T * b
    HtH = xxxxxx          # (2,2) - TODO: H^T H 행렬 완성
    Ht_b = xxxxxx         # (2,)  - TODO: H^T b 벡터 완성
    
    # 직접 역행렬을 써서 해 구하기: x_hat = (H^T H)^{-1} H^T b
    try:
        inv_HtH = np.linalg.inv(HtH)
    except np.linalg.LinAlgError:
        inv_HtH = np.linalg.pinv(HtH)
    line_params = xxxxxx # (2,) - TODO: inv_HtH와 Ht_b를 사용하여 line_params 완성
    
    return line_params

def custom_fit_sphere_from_samples(sampled_points: np.ndarray) -> Tuple[np.ndarray, float]:
    """샘플링된 포인트들로부터 구 피팅
    
    Args:
        sampled_points: 샘플링된 포인트들 (최소 4개)
    
    Returns:
        center: 구의 중심점
        radius: 구의 반지름
    """
    if len(sampled_points) < 4:
        raise ValueError("구 피팅을 위해서는 최소 4개의 포인트가 필요합니다.")
    
    x, y, z = sampled_points[:, 0], sampled_points[:, 1], sampled_points[:, 2]
    n = len(sampled_points)
    
    # 정규 방정식을 위한 행렬 H 와 관측값 b 구성
    # 정규 방정식을 위한 행렬 H 와 관측값 b 구성
    # H * [2cx, 2cy, 2cz, d]^T = b
    H = np.column_stack([x, y, z, np.ones(n)])  # TODO: H 행렬 완성
    b = x ** 2 + y ** 2 + z ** 2 # TODO: b 벡터 완성
    
    # 정규 방정식: H^T * H * params = H^T * b
    HtH = xxxxxx          # (4,4) - TODO: H^T H 행렬 완성
    Ht_b = xxxxxx         # (4,)  - TODO: H^T b 벡터 완성
    
    # 직접 역행렬을 써서 해 구하기: x_hat = (H^T H)^{-1} H^T b
    try:
        inv_HtH = np.linalg.inv(HtH)
    except np.linalg.LinAlgError:
        # 특이행렬일 때는 의사역행렬 사용
        inv_HtH = np.linalg.pinv(HtH)
    params = xxxxxx  # (4,) - TODO: inv_HtH와 Ht_b를 사용하여 line_params 완성
    
    # 파라미터 추출
    cx, cy, cz = xxxxxx, xxxxxx, xxxxxx # TODO: params에서 cx, cy, cz 추출
    d = xxxxxx # TODO: params에서 d 추출
    
    # 반지름 계산: r² = cx² + cy² + cz² + d
    radius_squared = cx**2 + cy**2 + cz**2 + d  # TODO: cx, cy, cz, d를 사용하여 radius_squared 계산
    radius0 = np.sqrt(max(0, radius_squared))  # 음수 방지
    center0 = np.array([cx, cy, cz], dtype=float)
    
    return center0, radius0

def custom_compute_line_distances(pcd: o3d.geometry.PointCloud, line_params: np.ndarray) -> np.ndarray:
    """모든 포인트와 직선 사이의 거리 계산
    
    직선 방정식: y = ax + b
    점-직선 거리: |ax - y + b| / √(a² + 1)
    
    Args:
        pcd: 포인트 클라우드
        line_params: 직선 파라미터 [a, b]
    
    Returns:
        distances: 각 포인트와 직선 사이의 거리
    """
    points = np.array(pcd.points)
    x, y = points[:, 0], points[:, 1]  # z는 사용하지 않음
    a, b = line_params
    
    # 점-직선 거리 공식 (직선: ax - y + b = 0)
    numerator = xxxxxx   # TODO: |ax - y + b| 계산
    denominator = xxxxxx # TODO: √(a² + 1) 계산
    
    distances = numerator / denominator
    
    return distances

def custom_compute_sphere_distances(pcd: o3d.geometry.PointCloud, center: np.ndarray, radius: float) -> np.ndarray:
    """모든 포인트와 구 표면 사이의 거리 계산
    
    구 표면까지의 거리: |√((x-cx)² + (y-cy)² + (z-cz)²) - r|
    
    Args:
        pcd: 포인트 클라우드
        center: 구의 중심점
        radius: 구의 반지름
    
    Returns:
        distances: 각 포인트와 구 표면 사이의 거리
    """
    points = np.array(pcd.points)
    cx, cy, cz = center
    
    # 각 포인트와 중심점 사이의 거리
    distances_to_center = xxxxxx  # TODO: np.linalg.norm 혹은 np.sqrt을 사용하여 각 포인트와 중심점 사이의 거리 계산
    
    # 구 표면까지의 거리
    distances = np.abs(distances_to_center - radius)
    
    return distances

def custom_find_inliers_outliers(distances: np.ndarray, threshold: float) -> Tuple[np.ndarray, np.ndarray]:
    """거리 기반으로 인라이어와 아웃라이어 분류
    
    Args:
        distances: 각 포인트의 거리
        threshold: 인라이어 판정 임계값
    
    Returns:
        inlier_indices: 인라이어 포인트들의 인덱스
        outlier_indices: 아웃라이어 포인트들의 인덱스
    """
    inlier_indices = np.where(distances < threshold)[0]
    outlier_indices = np.where(distances >= threshold)[0]
    
    return inlier_indices, outlier_indices

###############################################################################
# Visualization 함수들
def visualize_line_fitting_result(pcd: o3d.geometry.PointCloud, 
                                 line_params: np.ndarray, 
                                 inlier_indices: np.ndarray = None,
                                 outlier_indices: np.ndarray = None,
                                 text: str = None,
                                 text_coord: np.ndarray = None):
    """직선 피팅 결과 시각화
    
    Args:
        pcd: 원본 포인트 클라우드
        line_params: 직선 파라미터 [a, b]
        inlier_indices: 인라이어 포인트 인덱스 (선택사항)
        outlier_indices: 아웃라이어 포인트 인덱스 (선택사항)
        text: 텍스트 문자열 (선택사항)
        text_coord: 텍스트 좌표 (선택사항)
    """
    geometries = []
    
    # 포인트 클라우드 색상 설정
    pcd_copy = pcd.__copy__()
    points = np.array(pcd_copy.points)
    colors = np.ones((len(points), 3)) * 0.7  # 기본 회색
    
    if inlier_indices is not None:
        colors[inlier_indices] = [0, 1, 0]  # 인라이어: 초록색
    if outlier_indices is not None:
        colors[outlier_indices] = [1, 0, 0]  # 아웃라이어: 빨간색
    
    pcd_copy.colors = o3d.utility.Vector3dVector(colors)
    geometries.append(pcd_copy)
    
    # 피팅된 직선 생성 및 시각화
    a, b = line_params
    
    # 포인트 클라우드 x 범위 계산
    x_coords = points[:, 0]
    min_x = np.min(x_coords) - 1
    max_x = np.max(x_coords) + 1
    
    # 직선 점들 생성 (z=0 평면에서)
    x_range = np.linspace(min_x, max_x, 200)
    y_range = a * x_range + b
    z_range = np.zeros_like(x_range)  # z=0
    
    line_points = np.column_stack([x_range, y_range, z_range])
    
    # 선분으로 직선 표현
    line_pcd = o3d.geometry.PointCloud()
    line_pcd.points = o3d.utility.Vector3dVector(line_points)
    line_pcd.paint_uniform_color([0, 0, 1])  # 파란색
    geometries.append(line_pcd)
    
    # 좌표계 추가
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    geometries.append(coordinate_frame)

    # 텍스트 추가
    if text is not None and text_coord is not None:
        text_mesh = get_text_mesh(text, text_coord, text_size=0.05)
        geometries.append(text_mesh)
    
    o3d.visualization.draw_geometries(geometries)

def visualize_sphere_fitting_result(pcd: o3d.geometry.PointCloud, 
                                   center: np.ndarray, 
                                   radius: float,
                                   inlier_indices: np.ndarray = None,
                                   outlier_indices: np.ndarray = None,
                                   text: str = None,
                                   text_coord: np.ndarray = None):
    """구 피팅 결과 시각화
    
    Args:
        pcd: 원본 포인트 클라우드
        center: 구의 중심점
        radius: 구의 반지름
        inlier_indices: 인라이어 포인트 인덱스 (선택사항)
        outlier_indices: 아웃라이어 포인트 인덱스 (선택사항)
        text: 텍스트 문자열 (선택사항)
        text_coord: 텍스트 좌표 (선택사항)
    """
    geometries = []
    
    # 포인트 클라우드 색상 설정
    pcd_copy = pcd.__copy__()
    points = np.array(pcd_copy.points)
    colors = np.ones((len(points), 3)) * 0.7  # 기본 회색
    
    if inlier_indices is not None:
        colors[inlier_indices] = [0, 1, 0]  # 인라이어: 초록색
    if outlier_indices is not None:
        colors[outlier_indices] = [1, 0, 0]  # 아웃라이어: 빨간색
    
    pcd_copy.colors = o3d.utility.Vector3dVector(colors)
    geometries.append(pcd_copy)
    
    # 피팅된 구 생성 및 시각화
    sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
    sphere_mesh.translate(center)
    sphere_mesh.paint_uniform_color([0, 0, 1])  # 파란색
    sphere_mesh.compute_vertex_normals()
    
    # 와이어프레임 스타일로 변경
    sphere_wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(sphere_mesh)
    geometries.append(sphere_wireframe)
    
    # 좌표계 추가
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    geometries.append(coordinate_frame)
    
    # 텍스트 추가
    if text is not None and text_coord is not None:
        text_mesh = get_text_mesh(text, text_coord, text_size=0.05)
        geometries.append(text_mesh)
    
    o3d.visualization.draw_geometries(geometries)

def visualize_sampled_points(pcd: o3d.geometry.PointCloud, sampled_indices: np.ndarray, text: str, text_coord: np.ndarray):
    """샘플링된 포인트 시각화
    
    Args:
        pcd: 원본 포인트 클라우드
        sampled_indices: 샘플링된 포인트 인덱스
        text: 텍스트 문자열
        text_coord: 텍스트 좌표
    """
    geometries = []
            
    # 포인트 클라우드 색상 설정
    pcd_copy = pcd.__copy__()
    pcd_points = np.array(pcd_copy.points)
    pcd_colors = np.ones((len(pcd_points), 3)) * [0.7, 0.7, 0.7]
    pcd_colors[sampled_indices] = [0, 1, 0]
    pcd_copy.colors = o3d.utility.Vector3dVector(pcd_colors)
    geometries.append(pcd_copy)

    # 좌표계 추가
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    geometries.append(coordinate_frame)

    # 텍스트 추가
    text_mesh = get_text_mesh(text, text_coord, text_size=0.05)
    geometries.append(text_mesh)

    o3d.visualization.draw_geometries(geometries)