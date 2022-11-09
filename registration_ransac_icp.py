# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2018-2021 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------

import open3d as o3d
import numpy as np

import argparse


def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0,
                                             max_nn=30))
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5.0,
                                             max_nn=100))
    return (pcd_down, pcd_fpfh)

def refine_registration(source, target, result_ransac, distance_threshold, max_iterations):
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)

    return result

if __name__ == '__main__':
    pcd_data = o3d.data.DemoICPPointClouds()
    parser = argparse.ArgumentParser(
        'Global point cloud registration example with RANSAC')
    parser.add_argument('src',
                        type=str,
                        default=pcd_data.paths[0],
                        nargs='?',
                        help='path to src point cloud')
    parser.add_argument('dst',
                        type=str,
                        default=pcd_data.paths[1],
                        nargs='?',
                        help='path to dst point cloud')
    # voxel_size is in meter. The recommended value is 0.05 for a coarse scale and 0.0125 for a fine scale.
    parser.add_argument('--voxel_size',
                        type=float,
                        default=0.05,
                        help='voxel size in meter used to downsample inputs')
    parser.add_argument(
        '--distance_multiplier',
        type=float,
        default=1.5,
        help='multipler used to compute distance threshold'
        ' between correspondences.'
        'Threshold is computed by voxel_size * distance_multiplier.')
    parser.add_argument('--max_iterations',
                        type=int,
                        default=1000000,
                        help='number of max RANSAC iterations')
    parser.add_argument('--confidence',
                        type=float,
                        default=0.999,
                        help='RANSAC confidence')
    parser.add_argument(
        '--mutual_filter',
        action='store_true',
        help='whether to use mutual filter for putative correspondences')
    parser.add_argument(
        '--distance_threshold_icp',
        type=float,
        default=10,
        help='multipler used to compute distance threshold'
        ' between correspondences.'
        'Threshold is computed by voxel_size * distance_multiplier.')

    args = parser.parse_args()

    voxel_size = args.voxel_size
    distance_threshold = args.distance_multiplier * voxel_size

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    print('Reading inputs')
    src_mesh = o3d.io.read_triangle_mesh(args.src)
    dst = o3d.io.read_point_cloud(args.dst)
    src = src_mesh.sample_points_uniformly(number_of_points=len(dst.points))

    bbox_src = src.get_axis_aligned_bounding_box()
    bbox_dst = dst.get_axis_aligned_bounding_box()
    print(bbox_src.get_center(), bbox_src.get_extent())

    print('Downsampling inputs')
    src_down, src_fpfh = preprocess_point_cloud(src, voxel_size)
    dst_down, dst_fpfh = preprocess_point_cloud(dst, voxel_size)

    print('Running RANSAC')
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down,
        dst_down,
        src_fpfh,
        dst_fpfh,
        mutual_filter=args.mutual_filter,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.
        TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            #o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
            #    0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            args.max_iterations, args.confidence))

    src_down.paint_uniform_color([1, 0, 0])
    src.paint_uniform_color([1, 0, 0])
    dst_down.paint_uniform_color([0, 1, 0])
    dst.paint_uniform_color([0, 1, 0])

    np.save("results/ransac_transformation.npy",result_ransac.transformation)

    # display registration results
    print(o3d.pipelines.registration.evaluate_registration(src, dst, max_correspondence_distance=len(src.points), transformation=result_ransac.transformation))

    # show RANSAC transformation on downsampled point cloud
    o3d.visualization.draw([src_down.transform(result_ransac.transformation), dst_down])
    #o3d.visualization.draw([src.transform(result.transformation), dst])


    # refine registration with point-to-point ICP
    result_icp = o3d.pipelines.registration.registration_icp(
        src, dst, args.distance_threshold_icp, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=args.max_iterations))

    np.save("results/icp_transformation.npy",result_icp.transformation)

    # display registration results
    print(o3d.pipelines.registration.evaluate_registration(src, dst, max_correspondence_distance=len(src.points), transformation=result_icp.transformation))

    o3d.visualization.draw([src.transform(result_icp.transformation), dst])
