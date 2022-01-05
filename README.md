# Cross Covariance

NCKU Meclab package for fusing tracking objects (RSU and OBU).

## Introduction
This package fuses local perception from OBU and cooperative perception from RSU by using cross covariacne. 

### Reference
S. Matzka and R. Altendorfer, "A comparison of track-to-track fusion algorithms for automotive sensor fusion," 2008 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems, 2008, pp. 189-194, doi: 10.1109/MFI.2008.4648063. [paper](https://ieeexplore.ieee.org/document/4648063)

### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`/TODO`|`autoware_msgs::DetectedObjectArray`|Tracking object list from OBU.|
|`/TODO`|`autoware_msgs::DetectedObjectArray`|Tracking objects list from lilee RSU.|
|`/TODO`|`autoware_msgs::DetectedObjectArray`|Tracking objects list from CSIE RSU.|
|`/TODO`|`autoware_msgs::DetectedObjectArray`|Tracking objects list from wistron RSU.|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/t2t_fusion/cross_cov/objects`|`autoware_msgs::DetectedObjectArray`|the global tracking object list after cross covariance.|

