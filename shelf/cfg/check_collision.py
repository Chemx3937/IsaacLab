import open3d as o3d
import numpy as np


obj_path = "/home/ryz/env_object/env_object2/cob_gazebo_objects/Media/models/shelf_collision.obj"
mesh = o3d.io.read_triangle_mesh(obj_path)

if mesh.is_empty():
    print("OBJ 파일이 비어 있습니다.")
else:
    print("OBJ 파일이 정상적으로 로드되었습니다.")
    
    # 회전 행렬 생성 (90도 회전, z축이 위로 오게끔 정렬)
    rotation_matrix = mesh.get_rotation_matrix_from_xyz((np.pi *3/ 2, 0, 0))  # z축 기준으로 90도 회전
    mesh.rotate(rotation_matrix, center=(0, 0, 0))

    o3d.visualization.draw_geometries([mesh])
