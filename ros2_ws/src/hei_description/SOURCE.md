# Model provenance

Imported from local hei-rebot-lift at commit b7c45e1fd71e65e39b98eabb2f091210b3904d22 (plus its existing working tree), model HEI_robot_urdf.
Original source: https://github.com/lipengdong/hei-rebot-lift
The upstream Apache-2.0 license is retained in LICENSE.upstream.

Meshes and kinematic origins are preserved. Mesh URIs now use package://hei_description.
Zero velocity/effort limits on lift/finger joints were replaced with provisional mock-only values;
arm velocity limits are reduced to 0.5 rad/s, wheel mock cap 3 rad/s.
These are NOT measured actuator ratings or hardware calibration.
The ros2_control wrapper supports ONLY mock_components/GenericSystem.
