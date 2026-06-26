# Decisions: lidar_camera_projection

## Array index as point identity through the transform pipeline

**Date**: 2026-05-18

**Decision**: Use the array index of a point in `cloud_pixel` to map back to its corresponding point in `cloud_voxel`, eliminating the need for an explicit ID field or custom point type.

**Rationale**: `transformCloud` is 1:1 (N in, N out) and `transformCloudToPixel` was modified to also be 1:1 by removing boundary/depth filtering. Each pixel-space point at index `i` always corresponds to the voxel point at index `i`. This avoids introducing a custom PCL point type (`PointXYZIL`), parallel index vectors, or any extra bookkeeping.

**Alternatives considered**:
- **Custom PCL point type `PointXYZIL`**: Adds x, y, z, intensity, uint32_t label. PCL boilerplate (`POINT_CLOUD_REGISTER_POINT_STRUCT`), serialization overhead, and all downstream consumers must understand the new type. Rejected as unnecessary when 1:1 ordering already guarantees identity.
- **Parallel `std::vector<uint32_t>`**: Carried alongside each point cloud through the pipeline. Fragile — must stay in sync with point cloud through all resizes and reorders. Rejected due to dual-bookkeeping risk.

**Consequences**: `transformCloudToPixel` no longer performs boundary filtering (z <= 0.05 or outside-image clipping), so out-of-bounds pixel points remain in the cloud. Downstream consumers that previously relied on `cloud_pixel.size() < cloud_voxel.size()` must handle invalid pixel coordinates.

---

## 2D spatial grid for fast pixel-space point-in-box queries

**Date**: 2026-05-18

**Decision**: Use a uniform 2D grid (10px cell size default) to bucket pixel-space point indices, rather than a linear scan or kd-tree.

**Rationale**: Given a detection bounding box, we only need to check points in grid cells that overlap the box. Build time is O(N), query time is O(K) where K is the number of points in relevant cells (typically much less than N). A linear scan would be O(N) per box — fine for small clouds but degrades with more points or many boxes.

**Alternatives considered**:
- **Linear scan**: Simplest to implement. O(N) per query. Acceptable for <5000 points and a few boxes. Rejected because the grid adds negligible complexity and scales better.
- **PCL KdTreeFLANN (2D)**: O(N log N) build, O(log N + K) per range search. Works but is heavier to build per frame than the grid, and range queries are not as direct as cell iteration.
- **Hash map (float keys)**: Unordered map keyed by (u, v) pixel coords. Hash collisions from floating-point precision make it fragile.

**Consequences**: Introduces `libpoint_lookup.so` and `PointLookup` class. Cell size is configurable (default 10). Grid memory overhead is O(N + G) where G is the number of cells.

---

## Normalized detection bboxes queried directly (no intermediate pixel conversion)

**Date**: 2026-05-20

**Decision**: `camera_frame_node` reads `detections.json` directly and uses `PointLookup::queryBoxNormalized()` which scales [0,1] coordinates to pixel space internally. Removed `loadAndSaveDetections()` disk write, `detections_pixel.json`, and `loadDetectionsPixel()`.

**Rationale**: The normalized→pixel multiplication was a per-frame waste — done once in a file write, then again on file read. Working directly with [0,1] bboxes eliminates unnecessary disk I/O. The 2D grid remains pixel-based (correct for projected LiDAR points); only the bbox input path changed.

**Consequences**: `lidar_filter_node` no longer writes `detections_pixel.json`. The `detections_loader` library now only exports `loadDetectionsNormalized()`.

---

## Cluster tolerance as configurable ROS param

**Date**: 2026-05-20

**Decision**: `cluster_tolerance` exposed as a ROS param in `lidar_camera_transform.launch` (default 0.8), set via `nh_.param<double>("cluster_tolerance", lookup_.cluster_tolerance_, 0.8)`.

**Consequences**: Users can tune Euclidean clustering sensitivity without recompiling.
