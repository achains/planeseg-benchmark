import sys
import os
import shutil
import open3d as o3d

from tum_reader import read_depth
from organize_pcd import organize_point_cloud


def main():
    input_dir, output_dir = sys.argv[1:]
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    os.mkdir(output_dir, mode=0o777)

    for png in filter(lambda x: x.endswith(".png"), os.listdir(input_dir)):
        pcd = read_depth(input_dir + "/" + png)
        pcd_filename = png.strip(".png") + ".pcd"
        o3d.io.write_point_cloud(pcd_filename, pcd)
        org_pcd = organize_point_cloud(pcd_filename)
        org_pcd.save_pcd(output_dir + "/" + pcd_filename)
        os.remove(pcd_filename)


if __name__ == '__main__':
    main()
