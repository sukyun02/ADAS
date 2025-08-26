"""
 * @file        custom_camera.py
 * @brief       Custom Camera class for image processing and analysis.
 * 
 * @authors     Jaehwan Lee (idljh5529@gmail.com)      
 *
 * @date        2025-08-13 Released by AI Lab, Hanyang University
 * 
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
from typing import List, Tuple, Optional, Union
import glob

class CameraProcessor:
    """카메라 이미지 처리를 위한 클래스"""
    
    def __init__(self):
        """초기화"""
        self.image = None
        self.image_path = None
        
    def load_image(self, image_path: str, color_mode: str = 'BGR') -> np.ndarray:
        """
        이미지를 로드합니다.
        
        Args:
            image_path (str): 이미지 파일 경로
            color_mode (str): 'BGR', 'RGB', 'GRAY' 중 선택
            
        Returns:
            np.ndarray: 로드된 이미지
        """
        try:
            if color_mode.upper() == 'GRAY':
                image = cv2.imread(xxxxxx, xxxxxx) # TODO: cv2.IMREAD_GRAYSCALE 사용
            else:
                image = cv2.imread(image_path, cv2.IMREAD_COLOR)
                if color_mode.upper() == 'RGB' and image is not None:
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            if image is None:
                print(f"⚠ 이미지를 로드할 수 없습니다: {image_path}")
                # 시뮬레이션 이미지 생성
                if color_mode.upper() == 'GRAY':
                    image = self._generate_sample_image_gray()
                else:
                    image = self._generate_sample_image_color()
                    if color_mode.upper() == 'RGB':
                        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                print("시뮬레이션 이미지를 생성했습니다.")
            
            self.image = image
            self.image_path = image_path
            return image
            
        except Exception as e:
            print(f"이미지 로딩 중 오류 발생: {e}")
            return self._generate_sample_image_color()
    
    def _generate_sample_image_color(self) -> np.ndarray:
        """컬러 시뮬레이션 이미지를 생성합니다."""
        height, width = 480, 640
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 도로 배경
        image[int(height*0.6):, :] = [50, 50, 50]  # 회색 도로
        
        # 하늘 배경
        image[:int(height*0.6), :] = [135, 206, 235]  # 하늘색
        
        # 차선 그리기
        lane_y = int(height * 0.8)
        # 왼쪽 차선
        cv2.line(image, (int(width*0.3), height), (int(width*0.45), lane_y), (255, 255, 255), 3)
        # 오른쪽 차선
        cv2.line(image, (int(width*0.7), height), (int(width*0.55), lane_y), (255, 255, 255), 3)
        
        # 중앙선 (점선)
        for i in range(lane_y, height, 20):
            cv2.line(image, (int(width*0.5), i), (int(width*0.5), i+10), (255, 255, 0), 2)
        
        # 차량 추가
        cv2.rectangle(image, (int(width*0.6), int(height*0.7)), 
                     (int(width*0.8), int(height*0.85)), (0, 0, 255), -1)
        
        return image
    
    def _generate_sample_image_gray(self) -> np.ndarray:
        """그레이스케일 시뮬레이션 이미지를 생성합니다."""
        color_image = self._generate_sample_image_color()
        return cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
    def load_multiple_images(self, folder_path: str, extensions: List[str] = ['jpg', 'jpeg', 'png', 'bmp']) -> List[np.ndarray]:
        """
        폴더에서 여러 이미지를 로드합니다.
        
        Args:
            folder_path (str): 이미지 폴더 경로
            extensions (List[str]): 지원할 파일 확장자
            
        Returns:
            List[np.ndarray]: 로드된 이미지들의 리스트
        """
        images = []
        image_paths = []
        
        if not os.path.exists(folder_path):
            print(f"⚠ 폴더를 찾을 수 없습니다: {folder_path}")
            print("시뮬레이션 이미지들을 생성합니다.")
            # 다양한 환경의 시뮬레이션 이미지 생성
            return self._generate_sample_images()
        
        for ext in extensions:
            pattern = os.path.join(folder_path, f"*.{ext}")
            image_paths.extend(glob.glob(pattern))
            pattern = os.path.join(folder_path, f"*.{ext.upper()}")
            image_paths.extend(glob.glob(pattern))
        
        if not image_paths:
            print(f"⚠ 폴더에서 이미지를 찾을 수 없습니다: {folder_path}")
            return self._generate_sample_images()
        
        for path in sorted(image_paths):
            image = cv2.imread(path)
            if image is not None:
                images.append(image)
        
        print(f"✓ {len(images)}개의 이미지를 로드했습니다.")
        return images
    
    def _generate_sample_images(self) -> List[np.ndarray]:
        """다양한 환경의 시뮬레이션 이미지들을 생성합니다."""
        images = []
        
        # 1. 주간 이미지
        day_image = self._generate_sample_image_color()
        images.append(day_image)
        
        # 2. 야간 이미지 (어둡게)
        night_image = (day_image * 0.3).astype(np.uint8)
        # 헤드라이트 효과 추가
        cv2.circle(night_image, (int(640*0.2), int(480*0.7)), 50, (255, 255, 200), -1)
        cv2.circle(night_image, (int(640*0.8), int(480*0.7)), 50, (255, 255, 200), -1)
        images.append(night_image)
        
        # 3. 저녁 이미지 (주황빛)
        evening_image = day_image.copy()
        evening_image[:, :, 0] = np.clip(evening_image[:, :, 0] * 0.8, a_min=0, a_max=255)  # 파란색 채널 감소
        evening_image[:, :, 2] = np.clip(evening_image[:, :, 2] * 1.2, a_min=0, a_max=255)  # 빨간색 채널 증가
        images.append(evening_image)
        
        # 4. 흐린 날씨 (대비 감소)
        cloudy_image = cv2.convertScaleAbs(day_image, alpha=0.7, beta=30)
        images.append(cloudy_image)
        
        return images
    
    def display_image(self, image: np.ndarray, title: str = "Image", figsize: Tuple[int, int] = (10, 6)):
        """
        matplotlib를 사용하여 이미지를 표시합니다.
        
        Args:
            image (np.ndarray): 표시할 이미지
            title (str): 이미지 제목
            figsize (Tuple[int, int]): 그림 크기
        """
        plt.figure(figsize=figsize)
        
        if len(image.shape) == 3:
            # 컬러 이미지인 경우 BGR -> RGB 변환 (OpenCV는 BGR 순서)
            if image.shape[2] == 3:
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                plt.imshow(image_rgb)
            else:
                plt.imshow(image)
        else:
            # 그레이스케일 이미지
            plt.imshow(image, cmap='gray')
        
        plt.title(title)
        plt.axis('off')
        plt.tight_layout()
        plt.show()
    
    def display_multiple_images(self, images: List[np.ndarray], titles: List[str] = None, 
                              cols: int = 3, figsize: Tuple[int, int] = (15, 10)):
        """
        여러 이미지를 그리드로 표시합니다.
        
        Args:
            images (List[np.ndarray]): 표시할 이미지들
            titles (List[str]): 각 이미지의 제목
            cols (int): 열 개수
            figsize (Tuple[int, int]): 그림 크기
        """
        if not images:
            print("표시할 이미지가 없습니다.")
            return
            
        rows = (len(images) + cols - 1) // cols
        
        # Edge case 처리: 이미지가 1개이고 cols가 1인 경우
        if len(images) == 1 and cols == 1:
            fig, ax = plt.subplots(1, 1, figsize=figsize)
            axes = [ax]
        else:
            fig, axes = plt.subplots(rows, cols, figsize=figsize)
            
            # axes 배열 정규화
            if rows == 1 and cols == 1:
                axes = [axes]
            elif rows == 1 or cols == 1:
                axes = axes.flatten() if hasattr(axes, 'flatten') else [axes]
            else:
                axes = axes.flatten()
        
        for i, image in enumerate(images):
            if i < len(axes):
                if len(image.shape) == 3 and image.shape[2] == 3:
                    # BGR to RGB 변환
                    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    axes[i].imshow(image_rgb)
                elif len(image.shape) == 2:
                    # 그레이스케일
                    axes[i].imshow(image, cmap='gray')
                else:
                    axes[i].imshow(image)
                
                if titles and i < len(titles):
                    axes[i].set_title(titles[i])
                axes[i].axis('off')
        
        # 빈 subplot 숨기기
        for i in range(len(images), len(axes)):
            if i < len(axes):
                axes[i].axis('off')
        
        plt.tight_layout()
        plt.show()
    
    # ==================== 기본 이미지 처리 함수들 ====================
    
    def rgb_to_gray(self, image: np.ndarray) -> np.ndarray:
        """RGB/BGR 이미지를 그레이스케일로 변환합니다."""
        if len(image.shape) == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image
    
    def add_noise(self, image: np.ndarray, noise_type: str = 'gaussian', 
                  intensity: float = 25.0) -> np.ndarray:
        """
        이미지에 노이즈를 추가합니다.
        
        Args:
            image (np.ndarray): 입력 이미지
            noise_type (str): 'gaussian', 'salt_pepper', 'uniform'
            intensity (float): 노이즈 강도
            
        Returns:
            np.ndarray: 노이즈가 추가된 이미지
        """
        noisy_image = image.copy().astype(np.float32)
        
        if noise_type == 'gaussian':
            noise = np.random.normal(0, intensity, image.shape)
            noisy_image += xxxxxx # TODO: noise를 이미지에 추가
            
        elif noise_type == 'salt_pepper':
            # Salt and Pepper 노이즈
            prob = intensity / 100.0
            random_matrix = np.random.random(image.shape[:2])
            
            noisy_image[random_matrix < xxxxxx/2] = 0  # Pepper - TODO: 인자로 준 파라미터 값 적용
            noisy_image[random_matrix > 1 - xxxxxx/2] = 255  # Salt - TODO: 인자로 준 파라미터 값 적용
                
        elif noise_type == 'uniform':
            noise = np.random.uniform(-intensity, intensity, image.shape)
            noisy_image += xxxxxx # TODO: noise를 이미지에 추가
        
        return np.clip(noisy_image, a_min=0, a_max=255).astype(np.uint8)
    
    def remove_noise(self, image: np.ndarray, method: str = 'gaussian') -> np.ndarray:
        """
        이미지에서 노이즈를 제거합니다.
        
        Args:
            image (np.ndarray): 입력 이미지
            method (str): 'gaussian', 'median', 'bilateral'
            
        Returns:
            np.ndarray: 노이즈가 제거된 이미지
        """
        if method == 'gaussian':
            return cv2.GaussianBlur(image, (5, 5), 0)
        elif method == 'median':
            return cv2.medianBlur(image, 5)
        elif method == 'bilateral':
            return cv2.bilateralFilter(image, 9, 75, 75)
        else:
            return image
    
    def invert_image(self, image: np.ndarray) -> np.ndarray:
        """이미지를 반전시킵니다."""
        return cv2.bitwise_not(image)
    
    def adjust_contrast(self, image: np.ndarray, alpha: float = 1.5, beta: int = 0) -> np.ndarray:
        """
        이미지의 대비와 밝기를 조정합니다.
        
        Args:
            image (np.ndarray): 입력 이미지
            alpha (float): 대비 조정 (1.0은 원본, >1.0은 대비 증가)
            beta (int): 밝기 조정 (0은 원본, >0은 밝게)
            
        Returns:
            np.ndarray: 조정된 이미지
        """
        return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    
    def blur_image(self, image: np.ndarray, kernel_size: int = 15) -> np.ndarray:
        """이미지를 블러 처리합니다."""
        return cv2.GaussianBlur(image, (xxxxxx, xxxxxx), 0) # TODO: 가우시안 블러 함수 인자 작성 (커널 사이즈)
    
    def sharpen_image(self, image: np.ndarray) -> np.ndarray:
        """이미지를 샤픈 처리합니다."""
        kernel = np.array([[-1, -1, -1],
                          [-1,  9, -1],
                          [-1, -1, -1]])
        return cv2.filter2D(image, -1, kernel)
    
    def dilate_image(self, image: np.ndarray, kernel_size: int = 5, iterations: int = 1) -> np.ndarray:
        """이미지를 팽창시킵니다."""
        kernel = np.ones((xxxxxx, xxxxxx), np.uint8) # TODO: 팽창 함수 인자 작성 (커널 사이즈)
        return cv2.dilate(image, kernel, iterations=iterations)
    
    def erode_image(self, image: np.ndarray, kernel_size: int = 5, iterations: int = 1) -> np.ndarray:
        """이미지를 침식시킵니다."""
        kernel = np.ones((xxxxxx, xxxxxx), np.uint8) # TODO: 침식 함수 인자 작성 (커널 사이즈)
        return cv2.erode(image, kernel, iterations=iterations)
    
    # ==================== 고급 이미지 처리 함수들 ====================
    
    def binarize_image(self, image: np.ndarray, threshold: int = 127, 
                      method: str = 'simple') -> np.ndarray:
        """
        이미지를 이진화합니다.
        
        Args:
            image (np.ndarray): 입력 이미지 (그레이스케일)
            threshold (int): 임계값
            method (str): 'simple', 'adaptive_mean', 'adaptive_gaussian', 'otsu'
            
        Returns:
            np.ndarray: 이진화된 이미지
        """
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        if method == 'simple':
            _, binary = cv2.threshold(image, xxxxxx, 255, cv2.THRESH_BINARY) # TODO: 이진화 함수 인자 작성 (임계값)
        elif method == 'adaptive_mean':
            block_size = 11
            binary = cv2.adaptiveThreshold(image, 255, xxxxxx, 
                                         cv2.THRESH_BINARY, xxxxxx, 2) # TODO: cv2.ADAPTIVE_THRESH_MEAN_C 사용하여 이진화
        elif method == 'adaptive_gaussian':
            block_size = 11
            binary = cv2.adaptiveThreshold(image, 255, xxxxxx, 
                                         cv2.THRESH_BINARY, xxxxxx, 2) # TODO: cv2.ADAPTIVE_THRESH_GAUSSIAN_C 사용하여 이진화
        elif method == 'otsu':
            _, binary = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        else:
            _, binary = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY)
        
        return binary
    
    def morphology_operations(self, image: np.ndarray, operation: str = 'opening', 
                            kernel_size: int = 5) -> np.ndarray:
        """
        모폴로지 연산을 수행합니다.
        
        Args:
            image (np.ndarray): 입력 이미지 (이진 이미지)
            operation (str): 'opening', 'closing', 'gradient', 'tophat', 'blackhat'
            kernel_size (int): 커널 크기
            
        Returns:
            np.ndarray: 처리된 이미지
        """
        kernel = cv2.getStructuringElement(xxxxxx, (xxxxxx, xxxxxx)) # TODO: 커널을 어떠한 구조로 만들지 결정 (cv2.MORPH_RECT / cv2.MORPH_ELLIPSE / cv2.MORPH_CROSS 사용)
        
        if operation == 'opening':
            return cv2.morphologyEx(image, xxxxxx, xxxxxx) # TODO: cv2.MORPH_OPEN 사용 - 침식 후 팽창
        elif operation == 'closing':
            return cv2.morphologyEx(image, xxxxxx, xxxxxx) # TODO: cv2.MORPH_CLOSE 사용 - 팽창 후 침식
        elif operation == 'gradient':
            return cv2.morphologyEx(image, xxxxxx, xxxxxx) # TODO: cv2.MORPH_GRADIENT 사용 - 팽창 이미지와 침식 이미지의 차이
        elif operation == 'tophat':
            return cv2.morphologyEx(image, xxxxxx, xxxxxx) # TODO: cv2.MORPH_TOPHAT 사용 - 원본 이미지와 침식 이미지의 차이
        elif operation == 'blackhat':
            return cv2.morphologyEx(image, xxxxxx, xxxxxx) # TODO: cv2.MORPH_BLACKHAT 사용 - 팽창 이미지와 원본 이미지의 차이
        else:
            return image
    
    def edge_detection_sobel(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Sobel 엣지 검출을 수행합니다.
        
        Args:
            image (np.ndarray): 입력 이미지
            
        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray]: (x방향 엣지, y방향 엣지, 합성 엣지)
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Sobel 필터 적용
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, xxxxxx, xxxxxx, ksize=3) # TODO: 소벨 필터 적용 (x방향) - 소벨 필터 인자 작성 (https://deep-learning-study.tistory.com/205 참고)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, xxxxxx, xxxxxx, ksize=3) # TODO: 소벨 필터 적용 (y방향) - 소벨 필터 인자 작성
        
        # 절댓값으로 변환
        sobel_x = np.abs(xxxxxx) # TODO: 절댓값 함수 적용
        sobel_y = np.abs(xxxxxx) # TODO: 절댓값 함수 적용
        
        # 합성 엣지
        sobel_combined = xxxxxx # TODO: 합성 엣지 계산: 루트(x^2 + y^2)
        
        # 0-255 범위로 정규화
        sobel_x = np.clip(xxxxxx, a_min=0, a_max=255).astype(np.uint8) # TODO: 0-255 범위로 정규화
        sobel_y = np.clip(xxxxxx, a_min=0, a_max=255).astype(np.uint8) # TODO: 0-255 범위로 정규화
        sobel_combined = np.clip(xxxxxx, a_min=0, a_max=255).astype(np.uint8) # TODO: 0-255 범위로 정규화
        
        return sobel_x, sobel_y, sobel_combined
    
    def edge_detection_canny(self, image: np.ndarray, low_threshold: int = 50, 
                           high_threshold: int = 150) -> np.ndarray:
        """
        Canny 엣지 검출을 수행합니다.
        
        Args:
            image (np.ndarray): 입력 이미지
            low_threshold (int): 낮은 임계값
            high_threshold (int): 높은 임계값
            
        Returns:
            np.ndarray: 검출된 엣지
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        return cv2.Canny(gray, xxxxxx, xxxxxx) # TODO: Canny 함수 인자 작성 (낮은 임계값, 높은 임계값)
    
    def detect_contours(self, image: np.ndarray) -> Tuple[List, np.ndarray]:
        """
        윤곽선을 검출합니다.
        
        Args:
            image (np.ndarray): 입력 이미지 (이진 이미지)
            
        Returns:
            Tuple[List, np.ndarray]: (윤곽선 리스트, 윤곽선이 그려진 이미지)
        """
        contours, _ = cv2.findContours(image, xxxxxx, xxxxxx) # TODO: 윤곽선 검출 함수 인자 작성 (외곽선 검출 모드 - cv2.RETR_EXTERNAL / cv2.RETR_CCOMP, 근사화 방법 - cv2.CHAIN_APPROX_SIMPLE)
        
        # 원본 이미지가 그레이스케일이면 3채널로 변환
        if len(image.shape) == 2:
            contour_image = cv2.cvtColor(image, xxxxxx) # TODO: 그레이스케일 이미지를 3채널로 변환
        else:
            contour_image = image.copy()
        
        # 윤곽선 그리기
        cv2.drawContours(contour_image, xxxxxx, -1, (0, 255, 0), 2) # TODO: 윤곽선 그리기 함수 인자 작성 (윤곽선 이미지, 윤곽선 리스트, 윤곽선 인덱스, 색상, 두께)
        
        return contours, contour_image
    
    # ==================== 차선 검출 함수들 ====================
    
    def lane_detection_pipeline(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray, dict]:
        """
        기본적인 차선 검출을 수행합니다.
        
        Args:
            image (np.ndarray): 입력 이미지
            
        Returns:
            Tuple[np.ndarray, np.ndarray, dict]: (처리된 이미지, 차선이 검출된 이미지, 각 단계별 처리 결과)
        """
        results = {}

        # 원본 이미지
        results['original'] = image.copy()

        # 1단계: 그레이스케일 변환
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, xxxxxx) # TODO: 컬러 이미지를 그레이스케일로 변환
        else:
            gray = image
        results['gray'] = gray
        
        # 2단계: 가우시안 블러
        blurred = xxxxxx # TODO: 가우시안 블러 함수 사용 - cv2.GaussianBlur
        results['blurred'] = blurred
        
        # 3단계: Canny 엣지 검출
        edges = xxxxxx(xxxxxx, 50, 150) # TODO: Canny 함수 사용 - cv2.Canny
        results['edges'] = edges

        # 4단계: 관심 영역 설정 (사다리꼴 모양)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        
        # 사다리꼴 꼭짓점 정의 - TODO
        trapezoid = np.array([
            [int(width * xxxxxx), int(height * xxxxxx)], # 좌하
            [int(width * xxxxxx), int(height * xxxxxx)], # 좌상
            [int(width * xxxxxx), int(height * xxxxxx)], # 우상
            [int(width * xxxxxx), int(height * xxxxxx)]  # 우하
        ], dtype=np.int32)
        
        cv2.fillPoly(mask, [trapezoid], 255)
        masked_edges = xxxxxx # TODO: cv2.bitwise_and 사용하여 edges와 mask를 비트 연산 (https://engineer-mole.tistory.com/237 참고)
        results['masked'] = masked_edges
        
        # 5단계: 허프 변환으로 직선 검출
        hough_threshold = 50
        hough_min_line_length = 100
        hough_max_line_gap = 50
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, xxxxxx, 
                               xxxxxx, xxxxxx) # TODO: 허프 변환 함수 인자 작성
        
        # 6단계: 검출된 직선을 원본 이미지에 그리기
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        results['lines'] = line_image
        
        # 원본 이미지와 합성
        final_result = cv2.addWeighted(image, 0.8, line_image, 1.0, 0)
        results['final'] = final_result
        
        return masked_edges, final_result, results
    
    # ==================== BEV 변환 (IPM) 함수들 ====================
    
    def setup_camera_params(self, intrinsic_matrix: np.ndarray, distortion_coeffs: np.ndarray,
                           rotation_matrix: np.ndarray, translation_vector: np.ndarray):
        """
        카메라 파라미터를 설정합니다.
        
        Args:
            intrinsic_matrix (np.ndarray): 내부 파라미터 행렬 (3x3)
            distortion_coeffs (np.ndarray): 왜곡 계수 (5x1)
            rotation_matrix (np.ndarray): 회전 행렬 (3x3)
            translation_vector (np.ndarray): 평행이동 벡터 (3x1)
        """
        self.K = intrinsic_matrix
        self.D = distortion_coeffs
        self.R = rotation_matrix
        self.t = translation_vector
        
        # 외부 파라미터 행렬 [R|t] 생성
        self.extrinsic_matrix = np.hstack([self.R, self.t.reshape(-1, 1)])
        
        print("카메라 파라미터가 설정되었습니다.")
        print(f"내부 파라미터 행렬 K:\n{self.K}")
        print(f"외부 파라미터 행렬 [R|t]:\n{self.extrinsic_matrix}")
    
    def undistort_image(self, image: np.ndarray) -> np.ndarray:
        """
        이미지 왜곡을 보정합니다.
        
        Args:
            image (np.ndarray): 입력 이미지
            
        Returns:
            np.ndarray: 왜곡이 보정된 이미지
        """
        if not hasattr(self, 'K') or not hasattr(self, 'D'):
            print("⚠ 카메라 파라미터가 설정되지 않았습니다.")
            return image
        
        return xxxxxx # TODO: 왜곡 보정 함수 사용 - cv2.undistort, self.K, self.D 활용
    
    def world_to_image(self, world_points: np.ndarray) -> np.ndarray:
        """
        월드 좌표를 이미지 좌표로 변환합니다.
        
        Args:
            world_points (np.ndarray): 월드 좌표 (Nx3)
            
        Returns:
            np.ndarray: 이미지 좌표 (Nx2)
        """
        if not hasattr(self, 'K') or not hasattr(self, 'extrinsic_matrix'):
            raise ValueError("카메라 파라미터가 설정되지 않았습니다.")
        
        # 동차 좌표로 변환
        ones = np.ones((world_points.shape[0], 1))
        world_homogeneous = np.hstack([world_points, ones])
        
        # 카메라 좌표계로 변환
        camera_coords = (self.extrinsic_matrix @ world_homogeneous.T).T
        
        # 이미지 좌표계로 투영
        image_coords_homogeneous = (self.K @ camera_coords.T).T
        
        # 정규화
        image_coords = image_coords_homogeneous[:, :2] / image_coords_homogeneous[:, 2:3]
        
        return image_coords
    
    def image_to_world_ground(self, image_points: np.ndarray, z_ground: float = 0.0) -> np.ndarray:
        """
        이미지 좌표를 지면의 월드 좌표로 변환합니다.
        
        Args:
            image_points (np.ndarray): 이미지 좌표 (Nx2)
            z_ground (float): 지면의 높이 (월드 좌표계)
            
        Returns:
            np.ndarray: 월드 좌표 (Nx3)
        """
        if not hasattr(self, 'K') or not hasattr(self, 'extrinsic_matrix'):
            raise ValueError("카메라 파라미터가 설정되지 않았습니다.")
        
        # 내부 파라미터의 역행렬
        K_inv = np.linalg.inv(self.K)
        
        # 외부 파라미터 분해
        R = self.extrinsic_matrix[:, :3]
        t = self.extrinsic_matrix[:, 3]
        
        world_points = []
        
        for image_point in image_points:
            # 동차 이미지 좌표
            image_homogeneous = np.array([image_point[0], image_point[1], 1.0])
            
            # 정규화된 카메라 좌표
            normalized_coords = K_inv @ image_homogeneous
            
            # 지면과의 교점 계산
            # 월드 좌표: X_w = R^(-1) * (λ * normalized_coords - t)
            # 지면 조건: Z_w = z_ground
            # λ를 구하기 위해 Z_w = z_ground 조건 사용
            
            R_inv = np.linalg.inv(R)
            
            # Z_w = z_ground = (R_inv @ (λ * normalized_coords - t))[2]
            # z_ground = R_inv[2,:] @ (λ * normalized_coords - t)
            # z_ground = λ * (R_inv[2,:] @ normalized_coords) - (R_inv[2,:] @ t)
            # λ = (z_ground + R_inv[2,:] @ t) / (R_inv[2,:] @ normalized_coords)
            
            lambda_param = (z_ground + R_inv[2, :] @ t) / (R_inv[2, :] @ normalized_coords)
            
            # 월드 좌표 계산
            world_point = R_inv @ (lambda_param * normalized_coords - t)
            world_points.append(world_point)
        
        return np.array(world_points)
    
    def create_bev_transform(self, image_shape: Tuple[int, int], 
                           bev_width: float = 10.0, bev_height: float = 20.0,
                           bev_resolution: float = 0.05) -> Tuple[np.ndarray, Tuple[int, int]]:
        """
        BEV (Bird's Eye View) 변환 행렬을 생성합니다.
        
        Args:
            image_shape (Tuple[int, int]): 입력 이미지 크기 (height, width)
            bev_width (float): BEV 이미지의 실제 폭 (미터)
            bev_height (float): BEV 이미지의 실제 높이 (미터)
            bev_resolution (float): BEV 이미지의 해상도 (미터/픽셀)
            
        Returns:
            Tuple[np.ndarray, Tuple[int, int]]: (변환 행렬, BEV 이미지 크기)
        """
        # BEV 이미지 크기 계산
        bev_img_width = int(xxxxxx) # TODO: BEV 이미지 폭 계산
        bev_img_height = int(xxxxxx) # TODO: BEV 이미지 높이 계산
        bev_size = (bev_img_height, bev_img_width)
        
        # 월드 좌표계의 4개 점 정의 (지면 위의 사각형)
        # 차량 뒤축 기준으로 앞쪽 방향
        world_points = np.array([
            [0, -bev_height/2, 0],          # 좌하
            [0, bev_height/2, 0],           # 좌상  
            [bev_width, bev_height/2, 0],   # 우상
            [bev_width, -bev_height/2, 0]   # 우하
        ], dtype=np.float32)
        
        # 월드 좌표를 이미지 좌표로 변환
        image_points = self.world_to_image(world_points).astype(np.float32)
        
        # BEV 이미지의 대응점
        bev_points = np.array([
            [xxxxxx, xxxxxx],      # 좌하
            [xxxxxx, xxxxxx],      # 좌상
            [xxxxxx, xxxxxx],      # 우상
            [xxxxxx, xxxxxx]       # 우하
        ], dtype=np.float32)
        
        # 변환 행렬 계산
        transform_matrix = xxxxxx # TODO: 변환 행렬 계산 - cv2.getPerspectiveTransform, image_points, bev_points 활용
        
        print(f"BEV 변환 행렬이 생성되었습니다.")
        print(f"BEV 이미지 크기: {bev_size}")
        print(f"실제 영역: {bev_width}m × {bev_height}m")
        print(f"해상도: {bev_resolution}m/픽셀")
        
        return transform_matrix, bev_size
    
    def apply_bev_transform(self, image: np.ndarray, transform_matrix: np.ndarray, 
                          bev_size: Tuple[int, int]) -> np.ndarray:
        """
        이미지에 BEV 변환을 적용합니다.
        
        Args:
            image (np.ndarray): 입력 이미지
            transform_matrix (np.ndarray): 변환 행렬
            bev_size (Tuple[int, int]): BEV 이미지 크기
            
        Returns:
            np.ndarray: BEV 변환된 이미지
        """
        bev_image = xxxxxx # TODO: BEV 변환 함수 사용 - cv2.warpPerspective, transform_matrix, bev_size 활용
        return bev_image
    
    def detect_lanes_bev(self, bev_image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        BEV 이미지에서 차선을 검출합니다.
        
        Args:
            bev_image (np.ndarray): BEV 변환된 이미지
            
        Returns:
            Tuple[np.ndarray, np.ndarray]: (처리된 이미지, 차선이 검출된 이미지)
        """
        # 그레이스케일 변환
        if len(bev_image.shape) == 3:
            gray = cv2.cvtColor(bev_image, xxxxxx) # TODO: 컬러 이미지를 그레이스케일로 변환
        else:
            gray = bev_image
        
        # 이진화 (밝은 차선 추출)
        _, binary = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)
        
        # 모폴로지 연산으로 노이즈 제거
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        cleaned = xxxxxx # TODO: 모폴로지 연산 함수 사용 - cv2.morphologyEx, cv2.MORPH_OPEN 활용
        
        # 윤곽선 검출
        contours, _ = xxxxxx # TODO: 윤곽선 검출 함수 사용 - cv2.findContours, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE 활용
        
        # 결과 이미지 생성
        result = cv2.cvtColor(gray, xxxxxx) # TODO: 그레이스케일 이미지를 3채널로 변환
        
        # 차선으로 판단되는 윤곽선 필터링 및 그리기
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 50:  # 최소 면적 조건
                # 윤곽선의 경계 사각형
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = h / w if w > 0 else 0
                
                # 차선은 세로가 가로보다 길어야 함
                if aspect_ratio > 2:
                    cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
        
        return cleaned, result
    
    def demonstrate_bev_pipeline(self, image: np.ndarray) -> dict:
        """
        전체 BEV 파이프라인을 시연합니다.
        
        Args:
            image (np.ndarray): 입력 이미지
            
        Returns:
            dict: 각 단계별 결과
        """
        results = {}
        
        # 원본 이미지
        results['original'] = image.copy()
        
        # 왜곡 보정
        if hasattr(self, 'K') and hasattr(self, 'D'):
            undistorted = self.undistort_image(image)
            results['undistorted'] = undistorted
        else:
            undistorted = image
            results['undistorted'] = undistorted
        
        # BEV 변환
        if hasattr(self, 'K'):
            transform_matrix, bev_size = self.create_bev_transform(image.shape[:2])
            bev_image = self.apply_bev_transform(undistorted, transform_matrix, bev_size)
            results['bev'] = bev_image
            
            # BEV에서 차선 검출
            lane_binary, lane_result = self.detect_lanes_bev(bev_image)
            results['bev_lanes_binary'] = lane_binary
            results['bev_lanes_result'] = lane_result
        else:
            print("⚠ 카메라 파라미터가 설정되지 않아 BEV 변환을 수행할 수 없습니다.")
        
        return results
