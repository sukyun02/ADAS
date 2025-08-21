"""Point Cloud I/O - Auxiliary Visualization"""
"""
 * @file        auxiliary_visualization.py
 * @brief       Custom auxiliary visualization algorithms for point cloud processing
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)          
 *
 * @date        2025-07-28 Released by AI Lab, Hanyang University
 * 
"""

import numpy as np
import open3d as o3d
import time
from typing import List

def get_text_mesh(text: str, text_coord: np.ndarray, text_size: float = 0.05) -> o3d.geometry.TriangleMesh:
    """텍스트 메시 생성
    
    Args:
        text: 텍스트 문자열
        text_coord: 텍스트 좌표
        text_size: 텍스트 크기
    """
    text_mesh = o3d.t.geometry.TriangleMesh.create_text(text, depth=1).to_legacy()

    # 스케일로 폰트 크기 조정
    text_mesh.scale(text_size, center=np.zeros(3))

    # 원하는 위치로 이동
    text_mesh.translate(text_coord)

    # 텍스트 색상 설정
    text_mesh.paint_uniform_color([50/255, 205/255, 50/255])

    # 법선 재계산
    text_mesh.compute_triangle_normals()

    # 복제하여 뒤집힌 면도 함께 그리기
    triangles = np.asarray(text_mesh.triangles)

    # 뒤집힌 삼각형 추가
    rev_triangles = triangles[:, ::-1]
    all_triangles = np.vstack([triangles, rev_triangles])
    text_mesh.triangles = o3d.utility.Vector3iVector(all_triangles)
    
    return text_mesh

def custom_create_bounding_box_mesh(center: np.ndarray, 
                                   extent: np.ndarray, 
                                   rotation: np.ndarray,
                                   color: List[float] = [1, 0, 0]) -> o3d.geometry.TriangleMesh:
    """바운딩 박스 메시 생성
    
    Args:
        center: 바운딩 박스 중심점
        extent: 바운딩 박스 크기 [width, height, depth]
        rotation: 바운딩 박스 회전 행렬 (3x3)
        color: 바운딩 박스 색상 [R, G, B]
    
    Returns:
        box_mesh: 바운딩 박스 메시
    """
    # 단위 박스 생성
    box_mesh = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
    
    # 박스를 원점 중심으로 이동 (기본적으로 한 모서리가 원점에 있음)
    box_mesh.translate([-0.5, -0.5, -0.5])
    
    # 크기 조정 - 각 축별로 개별적으로 스케일링
    vertices = np.asarray(box_mesh.vertices)
    vertices[:, 0] *= extent[0]  # x축 스케일링
    vertices[:, 1] *= extent[1]  # y축 스케일링
    vertices[:, 2] *= extent[2]  # z축 스케일링
    box_mesh.vertices = o3d.utility.Vector3dVector(vertices)
    
    # 회전 적용
    box_mesh.rotate(rotation, center=[0, 0, 0])
    
    # 최종 위치로 이동
    box_mesh.translate(center)
    
    # 색상 설정
    box_mesh.paint_uniform_color(color)
    
    # 법선 벡터 계산
    box_mesh.compute_vertex_normals()
    
    return box_mesh

def custom_create_bounding_box_wireframe(center: np.ndarray, 
                                        extent: np.ndarray, 
                                        rotation: np.ndarray,
                                        color: List[float] = [1, 0, 0]) -> o3d.geometry.LineSet:
    """바운딩 박스 와이어프레임 생성
    
    Args:
        center: 바운딩 박스 중심점
        extent: 바운딩 박스 크기 [width, height, depth]
        rotation: 바운딩 박스 회전 행렬 (3x3)
        color: 와이어프레임 색상 [R, G, B]
    
    Returns:
        wireframe: 바운딩 박스 와이어프레임
    """
    # 먼저 메시 생성
    box_mesh = custom_create_bounding_box_mesh(center, extent, rotation, color)
    
    # 메시를 와이어프레임으로 변환
    wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(box_mesh)
    wireframe.paint_uniform_color(color)
    
    return wireframe

def custom_wait(vis, duration=1.0, fps=60):
    interval = 1.0 / fps
    end_t = time.time() + duration
    while time.time() < end_t:
        vis.poll_events()
        vis.update_renderer()
        time.sleep(interval)