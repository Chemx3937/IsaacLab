import open3d as o3d
import os

def obj_to_stl(obj_path, stl_path):
    """
    OBJ 파일을 STL 파일로 변환하며, 노말을 계산합니다.

    Args:
        obj_path (str): 변환할 OBJ 파일 경로.
        stl_path (str): 변환된 STL 파일을 저장할 경로.
    """
    try:
        # OBJ 파일 읽기
        print(f"OBJ 파일 읽는 중: {obj_path}")
        mesh = o3d.io.read_triangle_mesh(obj_path)

        # 메쉬가 비어 있는지 확인
        if mesh.is_empty():
            print("에러: OBJ 파일이 비어 있습니다. 파일 내용을 확인하세요.")
            return

        # 노말 계산
        print("노말 계산 중...")
        mesh.compute_vertex_normals()

        # STL 파일로 저장
        print(f"STL 파일로 변환 중: {stl_path}")
        o3d.io.write_triangle_mesh(stl_path, mesh)
        print(f"변환 완료: {stl_path}")

    except Exception as e:
        print(f"에러 발생: {e}")

if __name__ == "__main__":
    obj_path = "/home/ryz/env_object/env_object2/cob_gazebo_objects/Media/models/shelf_collision.obj"
    stl_path = "/home/ryz/env_object/env_object2/cob_gazebo_objects/Media/models/shelf_collision.stl"

    if not os.path.exists(obj_path):
        print(f"에러: OBJ 파일이 존재하지 않습니다: {obj_path}")
    else:
        obj_to_stl(obj_path, stl_path)
