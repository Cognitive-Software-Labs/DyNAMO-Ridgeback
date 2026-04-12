from __future__ import annotations


PRIMARY_METRIC_KEYS = {
    'rgb': 'rgb_distance_m',
    'sensor_depth': 'sensor_depth_distance_m',
    'mono_depth': 'mono_depth_distance_m',
    'lidar': 'lidar_distance_m',
    'pointcloud': 'pointcloud_distance_m',
}

PRIMARY_METRIC_LABELS = {
    'rgb': 'RGB',
    'sensor_depth': 'Sensor depth',
    'mono_depth': 'Mono depth',
    'lidar': 'LiDAR',
    'pointcloud': 'Point-cloud',
}
