import pybullet as p
import glob

# 파일 경로 설정
file_dirs = glob.glob('/home/ryz/env_object/env_object2/cob_gazebo_objects/Media/models')

for file_dir in file_dirs:
    # PyBullet 초기화
    p.connect(p.DIRECT)

    # 입력 및 출력 파일 경로
    mesh_in = file_dir + "/shelf.obj"
    mesh_out = file_dir + "/shelf_collision.obj"
    log_name = 'log.txt'

    # VHACD 설정 및 실행
    p.vhacd(
        mesh_in,
        mesh_out,
        log_name,
        resolution=500000,          # 더 높은 해상도 (기본값: 100,000)
        depth=32,                   # 최대 클리핑 단계 증가 (기본값: 20)
        concavity=0.001,            # 오목도 허용치 (기본값: 0.0025 -> 더 정밀하게)
        planeDownsampling=2,        # 클리핑 평면 샘플링 더 세밀하게 (기본값: 4)
        convexhullDownsampling=2,   # 볼록체 생성 단계 세밀화 (기본값: 4)
        alpha=0.05,                 # 대칭성 클리핑 우선 순위
        beta=0.05,                  # 회전 축 기준 클리핑 우선 순위
        gamma=0.0005,               # 병합 단계에서 오목도 제한
        maxNumVerticesPerCH=128,    # 각 볼록체당 최대 정점 수 (기본값: 64 -> 128로 증가)
        minVolumePerCH=0.0001,      # 각 볼록체 최소 볼륨 (기본값 유지)
        pca=0,                      # PCA 기반 정규화 비활성화 (기본값: 0)
        mode=0,                     # Voxel-based (기본 모드)
        convexhullApproximation=1   # 볼록체 근사 사용 (기본값: 1)
    )
    print(f"충돌 메쉬 생성 완료: {mesh_out}")
