import open3d as o3d
import os

def stl_to_obj(stl_path, obj_path):
    """
    STL 파일을 OBJ 파일로 변환합니다.

    Args:
        stl_path (str): 변환할 STL 파일의 경로.
        obj_path (str): 변환된 OBJ 파일을 저장할 경로.
    """
    try:
        # STL 파일 읽기
        print(f"STL 파일 읽는 중: {stl_path}")
        mesh = o3d.io.read_triangle_mesh(stl_path)

        # 메쉬가 비어 있는지 확인
        if mesh.is_empty():
            print("에러: STL 파일이 비어 있습니다. 파일 내용을 확인하세요.")
            return

        # OBJ 파일로 저장
        print(f"OBJ 파일로 변환 중: {obj_path}")
        o3d.io.write_triangle_mesh(obj_path, mesh)
        print(f"변환 완료: {obj_path}")

    except Exception as e:
        print(f"에러 발생: {e}")

if __name__ == "__main__":
    # STL 및 OBJ 파일 경로 설정
    stl_path = "/home/ryz/env_object/env_object2/cob_gazebo_objects/Media/models/shelf.stl"
    obj_path = "/home/ryz/env_object/env_object2/cob_gazebo_objects/Media/models/shelf.obj"

    # 경로가 존재하는지 확인
    if not os.path.exists(stl_path):
        print(f"에러: STL 파일 경로가 존재하지 않습니다: {stl_path}")
    else:
        # STL -> OBJ 변환
        stl_to_obj(stl_path, obj_path)
