# Progress: lidar_camera_projection

## Completed
- **2026-05-18**: Implemented `PointLookup` class with 2D spatial grid — `include/lidar_camera_projection/point_lookup.hpp`, `src/point_lookup.cpp`
- **2026-05-18**: Added `loadDetectionsPixel()` to read `detections_pixel.json` — `src/detections_loader.cpp`, `detections_loader.hpp`
- **2026-05-18**: Integrated `PointLookup` and detection loading into `camera_frame_node` — publishes `/livox/inbox_voxel`
- **2026-05-18**: Modified `transformCloudToPixel` to preserve all points (no boundary/depth filtering) — ensures 1:1 correspondence between `cloud_pixel` and `cloud_voxel`

## In Progress
- **Point-in-box query**: Working end-to-end. Needs testing with live data to validate voxel-point correspondence.

## Blocked
- None

## Next Steps
1. Test with live LiDAR data and real detection outputs — verify inbox points match expected 3D regions
2. Profile grid query latency under load (10Hz frames, multiple detection boxes)
3. Consider adding `~detections_path` ROS param to launch files
4. Simplify filtering pipeline (per ARCHITECTURE.md TODO)

## Milestones
- [x] Point lookup infrastructure (grid + query + integration) — completed 2026-05-18
- [ ] Verified end-to-end with live data (target: TBD)
- [ ] Detection box visualization overlays (target: TBD)
