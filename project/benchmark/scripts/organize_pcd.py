from pypcd import pypcd
import numpy as np

import sys


def organize_point_cloud(filename: str):
    pcd = pypcd.PointCloud.from_path(filename)

    data = pcd.pc_data.view(np.float32)
    meta = pcd.get_metadata()
    meta["height"], meta["width"] = (480, 640)
    pcd = pypcd.make_xyz_point_cloud(xyz=data, metadata=meta)

    return pcd


def main():
    input_path, output_path = sys.argv[1:]
    organized_pcd = organize_point_cloud(input_path)
    organized_pcd.save_pcd(output_path)


if __name__ == "__main__":
    main()
