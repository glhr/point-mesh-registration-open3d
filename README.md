

# script

```
python3 registration_ransac_icp.py testData/source/Wave_mesh.ply testData/target/Wave.ply --voxel_size 50 --distance_multiplier 1.5 --distance_threshold_icp 10
```

(note: the voxel size and distance multiplier (for RANSAC) and the ICP distance threshold are imporant parameters. try playing around with them and see how it affects the fitness score & RMSE)

# results

RANSAC:

![ransac](results/ransac.png)

RegistrationResult with fitness=1.000000e+00, inlier_rmse=2.126101e+01, and correspondence_set size of 69543

ICP:

![icp](results/icp.png)

RegistrationResult with fitness=1.000000e+00, inlier_rmse=6.011599e+00, and correspondence_set size of 69543
