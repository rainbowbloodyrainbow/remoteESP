# Vendored micro-ROS component

- Repository: https://github.com/micro-ROS/micro_ros_espidf_component
- Distribution/branch: `humble`
- Component version: `22.0.0`
- Commit: `4ddd8c26e721662319ed8af981cb7cdc9ae05382`
- Retrieved: 2026-10-23
- License: Apache-2.0; original LICENSE/NOTICE and source remain in the component.
- Upstream manifest requires ESP-IDF >=5.2; local SDK is v5.2.7.
- Application disables the example netif source via CMake options and implements
  native STA lifecycle/reconnects in `main/remoteESP.c`.
- Local patch to component `CMakeLists.txt`: pass the archive's real path to
  `add_prebuilt_library` and remove the dependency on the undefined target
  `libmicroros_project`. Upstream already builds the archive synchronously with
  `execute_process`; the local IDF `add_prebuilt_library` does not create such a
  target. Runtime component sources are unchanged.

This is a source vendor copy, not an assumed prebuilt library. Its upstream
`libmicroros.mk` fetches and builds Humble ROS dependencies during the first
configure/build. Those transitive branches are upstream-managed, not all pinned
by the component commit. Network access and the documented Python dependencies
are still required.

The project provides the Humble `tf2_msgs/msg/TFMessage` interface in
`extra_ros_packages/tf2_msgs`. The local makefile extension incrementally builds
its C type support and repacks the prebuilt library, including when adding it
to an existing micro-ROS workspace; it does not delete the workspace.
