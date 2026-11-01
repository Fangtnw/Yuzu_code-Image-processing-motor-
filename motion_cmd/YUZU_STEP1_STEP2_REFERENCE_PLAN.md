# Yuzu Steps 1 and 2: reference calibration and integration notes

Recorded: 2026-10-28 (Asia/Tokyo).
Measurements planned: 2026-10-29.
Company review: Tuesday, 2026-11-10, 13:30.

## Agreed scope

- Focus on Steps 1 and 2. The horizontal-slider actuator has not been delivered; Step 3 is deferred.
- The detection team has already implemented detection in another repository. Reuse its results rather than reimplementing detection.
- Assume detection input comes from `/yuzu/yellow_dot_offsets`.
- This note records the integration approach. No automatic motion or calibration has been implemented by writing this note.

## Temporary Step 1 target

The existing input contract is `std_msgs/msg/Float32MultiArray`: 33 values containing 11 triples `[angle_deg, dx_mm, dy_mm]`, with angles 30, 60, ..., 330 degrees. Coordinates are relative to the detected yuzu center, X right and image vertical up. Image vertical must be mapped to machine Z through calibration.

The actual red 0-degree point is not included. For now, select the triple whose angle is 30 degrees and use its vertical offset as the temporary Step 1 insertion target. Do not change its angle identity in the Step 2 contour data. Replace this selection with the real red target when the detection team publishes it.

The selected point is already inset 3 mm toward the center. Do not subtract another 3 mm. A radial inset at 30 degrees is not exactly 3 mm vertically below the true top; this is an agreed temporary approximation, not a verified top-depth measurement.

Motor 1 moves vertically only. Ignore the selected point's horizontal offset for this movement; horizontal alignment is outside the current scope.

## Common reference for camera and holder

Proposed datum: conveyor plane is Z = 0, positive upward. Confirm whether the conveyor plane or a repeatable tray surface is the most practical datum during measurement. If the tray surface is used, record its height relative to the conveyor and use one consistent datum throughout.

The holder reference is the marked reference line on the holder in the Step 1 diagram, not the pin tips. Measure the installed reference line and record exactly which physical feature it represents.

At a known Motor 1 feedback position `q_ref_mm`, measure the holder reference height `z_holder_ref_mm` above the chosen datum. With verified positive-up Motor 1 coordinates and 1 mm of travel per commanded mm:

```text
z_target_mm = z_center_mm + dz_selected_mm
q_target_mm = q_ref_mm + (z_target_mm - z_holder_ref_mm)
```

Verify the installed axis direction and scaling before using this equation. If the command direction differs, apply the verified sign in the mapping.

Important: centroid-relative offsets alone do not provide `z_center_mm`. We need the detected center height relative to the datum, or an absolute target image coordinate with a calibrated camera-to-machine transform. Do not assume center height from the bottom contour or fruit radius without validating tray contact and geometry. A fixed center height is only suitable for an explicitly controlled test fixture.

## Measurement checklist for 2026-10-29

| Measurement / check | Value / observation |
|---|---|
| Chosen datum and exact physical surface | TBD |
| Tray surface height above conveyor, if applicable | TBD mm |
| Exact physical holder reference feature | TBD |
| Motor 1 feedback at calibration (`q_ref_mm`) | TBD mm |
| Holder reference height at calibration (`z_holder_ref_mm`) | TBD mm |
| Positive Motor 1 command moves physically upward or downward | TBD |
| Verified physical travel per commanded mm | TBD |
| Camera mounting: fixed or moving with Motor 1 | TBD |
| Camera-to-datum transform / image scale at the fruit position | TBD |
| Source of absolute yuzu center height (`z_center_mm`) | TBD |
| Holder reference-to-pin-tip distance, for interpreting insertion | TBD mm |
| Usable Motor 1 travel and mechanical clearance for insertion/lift | TBD |
| Lift definition: relative travel or final clearance above conveyor | TBD |
| Selected lift within the 100-150 mm requirement | TBD mm |
| Position tolerance and settling criteria | TBD |

Record the reference pose, encoder feedback, units, and a photo or sketch together so the calibration can be reproduced after restart or mechanical adjustment. Camera mounting determines whether the image transform must account for Motor 1 position at capture.

## Intended Step 1-2 sequence

1. Receive and validate the detection result for the fruit on the tray.
2. Select the temporary 30-degree target and convert it to a Motor 1 position using the measured reference mapping.
3. Command the holder insertion position through the existing guarded Motor 1 interface. Confirm arrival from feedback rather than treating publication as completion.
4. Lift the held fruit according to the confirmed 100-150 mm interpretation and measured clearances.
5. Acquire the Step 2 image with stationary acquisition dwell no greater than 100 ms. Resume the planned actuator motion while image processing continues. Confirm the subsequent motion and capture pose during implementation; do not invent an additional movement solely to satisfy this note.
6. Consume the contour result to calculate X/Z dimensions, center, and peeler tracking targets. Results must be ready before any dependent movement.

The 100 ms limit applies to image acquisition dwell; it does not state that all processing must complete within 100 ms. The current contour array has no capture timestamp, fruit ID, or acquisition-complete signal. Integration must establish how to associate results with the current capture and release the acquisition dwell independently of processing completion. Receiving a contour message alone does not prove acquisition timing.

## Existing integration points

- `motor_controller/yuzu_peeler_gui.py`: current manual controls and operator sequence; approach positions are entered manually.
- `motor_controller/azd3a_motor1_position_guard.py`: Motor 1 entry point, delegating to `azd3a_axis1_command_guard.py`.
- `MOTOR_ROS2_INTERFACE.md`: public guarded command and feedback contract; use this rather than raw controller topics.
- `motor_controller/yuzu_contour_profile.py`: validates the 11-ray contour and computes feed targets.
- `motor_controller/yuzu_contour_preview.py`: consumes `/yuzu/yellow_dot_offsets` and publishes calculated targets without commanding motors.
- `YUZU_CONTOUR_PREVIEW.md` and `YUZU_REAL_MOTOR_HANDOFF.md`: contour contract, provisional geometry, and coordination limitations.

Next implementation work depends on the measured reference mapping and the absolute center-height source. Preserve the existing guards and validate calculated targets against the configured travel envelope. This note does not establish calibration accuracy, completed integration, physical insertion depth, or timing compliance.
