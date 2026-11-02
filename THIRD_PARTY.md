# Attribution and licenses

This server repository combines original Mon-D work with the components below. Licenses apply to their respective components; the third-party notices do not assign a blanket license to all project files.

## Original project work

Mon-D-specific model integration, joint-state handling, and mapping/localization configuration are maintained as project sources. The existing `mondy_description` manifest declares Apache-2.0.

## JPL Open Source Rover

Source: [nasa-jpl/osr-rover-code](https://github.com/nasa-jpl/osr-rover-code/tree/e7c9e87bcd0fad273641bf0e7b67254dacdc924d).

Includes the OSR-derived model and meshes (`meshes/osr`, `urdf/osr_model.macro.xacro`, `urdf/insert_transmission.urdf.xacro`) and `src/osr_interfaces`. Local checkout revision: `e7c9e87bcd0fad273641bf0e7b67254dacdc924d`.

Copyright 2018 California Institute of Technology. Government Sponsorship Acknowledged. Apache License 2.0; the original copyright and full license are preserved in [licenses/JPL-OSR-LICENSE.txt](licenses/JPL-OSR-LICENSE.txt). No separate upstream NOTICE file was present in the inspected checkout.

The Mon-D model is modified: package mesh paths, camera/arm integration, base-frame TF changes, and removal of upstream Gazebo integration. Some model/mesh changes predate this packaging pass; the checkout revision identifies the local upstream source, not a claim that every derived file is unchanged.

## Waveshare RoArm M3

Source: [waveshareteam/roarm_ws](https://github.com/waveshareteam/roarm_ws/tree/40dbd84b553695212fab713e8465f817ba95454d/src/roarm_main/roarm_description).

Includes `meshes/roarm_m3`, `urdf/roarm_m3.macro.xacro`, and `urdf/roarm_m3/`. Local checkout revision: `40dbd84b553695212fab713e8465f817ba95454d`. The arm description was adapted for integration into the Mon-D model.

The upstream `roarm_description` manifest declares MIT. The original [package manifest](licenses/roarm_description.package.xml) and [MIT terms](licenses/RoArm-MIT-LICENSE.txt) are included. No standalone copyright notice or license file for this package was supplied by that checkout; no copyright year or ownership statement has been invented. Licenses from unrelated MoveIt packages in the workspace are not applied to these assets.

## External runtime dependencies

The OAK-D-PRO mesh is referenced from `depthai_descriptions` and is not redistributed here. RTAB-Map, ROS 2, robot_localization, Cyclone DDS, NumPy and Trimesh are external runtime/development dependencies; their implementation sources are not bundled. The robot's hardware code is maintained in a separate repository.
