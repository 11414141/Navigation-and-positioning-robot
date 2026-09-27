# Public Export Privacy Review

## Deliberately excluded

- Raw bags, PCD maps, PGM maps, RTK anchors, trajectories, field-site directories, and semantic waypoint files.
- Historical backups and modification logs from the robot. They contain machine paths and may contain operational context.
- The private ZED-F9P/NTRIP ROS launch file and all credentials it may use.
- Local Livox JSON configuration files containing the robot's deployed network addresses. A template is supplied instead.
- ROS build/install/log outputs and Python cache files.

## Included with safeguards

- RTK scripts retain their workflow logic but read `GNSS_PRIVATE_LAUNCH` from the local environment. They fail closed when it is not configured.
- Hardware names such as ROS topic names and `/dev/wheeltec_gnss` remain because they are integration interfaces, not credentials.
- Upstream package maintainer emails are retained only where they are part of third-party upstream package metadata.

## Operator responsibility

Inspect every staged change before pushing. Do not add local replacement configuration, site directories, or captured data without a fresh privacy review.
