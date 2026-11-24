# Camera measurement: projective validity and limits

The browser camera workflow uses four manually selected corners of a known rectangular reference and four corners of the opening. It applies a planar homography. It is not a depth sensor, LiDAR scan, or calibrated photogrammetry system.

## Coordinate normalization and conditioning

Before solving the 8-parameter homography system, both source image points and destination reference points are translated to their centroid and scaled so their mean distance from the origin is approximately √2. The transform is solved in these normalized coordinates and then denormalized.

The implementation estimates the infinity-norm condition number of the normalized 8×8 linear system by solving for the inverse matrix. A non-finite estimate or a value above 1e7 rejects calibration. Values above 1e5 are treated as a warning: the provisional dimensions may be shown for diagnosis, but the measured gate width is not copied into the simulator settings. The user must enlarge the reference in the image, improve its orientation/placement, and calibrate again.

These thresholds are numerical fail-closed heuristics, not validated camera-accuracy limits. A low condition estimate only indicates that this particular linear system is not severely ill-conditioned; it does not validate lens calibration, coplanarity, corner selection, or the physical reference dimensions.

## Projective-horizon guard

For a homography with denominator

`w(x,y) = g x + k y + 1`

the denominator is affine over image coordinates. The measured opening is accepted only when all four corners have finite denominators, remain farther than a scale-relative numerical tolerance from zero, and have the same sign.

Because the selected opening must be a convex quadrilateral, equal non-zero signs at all four vertices mean the projective horizon does not cross its interior. Mixed signs or a near-zero corner cause a fail-closed error rather than returning potentially unbounded dimensions.

This guard detects one class of mathematically invalid extrapolation. It does not prove that the reference and opening are coplanar or that the camera intrinsics are calibrated.

## Measurement limitations

The reported 1σ value remains a first-order sensitivity estimate under independent coordinate-error assumptions. It does not include, among other things:

- lens distortion and rolling-shutter effects;
- camera motion between reference and opening selection;
- non-planar or non-coplanar surfaces;
- inaccurate reference dimensions or damaged reference edges beyond the entered uncertainty;
- glare, blur, occlusion, and ambiguous corners;
- systematic touch-coordinate or video-crop offsets.

On successful map registration, the local width sensitivity estimate is passed to the gate geometry as a separate full-width envelope of ±2σ. The planner narrows the modeled clear opening by that amount; this is only an approximate one-dimensional normal assumption, not a deterministic bound or validated 95% guarantee. It does not reduce or replace the manually entered general clearance reserve. The map center and gate heading are still entered manually, with separate per-axis position and angular uncertainty fields. Verify dimensions with a tape measure, independently establish the map anchor and heading, and use a conservative manually selected clearance reserve.

## iPhone field check

1. Open the HTTPS application in Safari and explicitly tap **Включить камеру**.
2. Deny permission once and confirm that a clear explanation appears and the simulator remains usable.
3. Grant permission, place the reference and opening in the same plane, and do not move the phone.
4. Measure the opening, then repeat after changing the phone angle and distance.
5. Compare every result with a tape measure; record the difference rather than treating the displayed 1σ as total accuracy.
6. Confirm that orientation changes, video-frame dimension changes, stopping the stream, or changing reference dimensions invalidate calibration.
7. Confirm that a projective-horizon error is shown as a failed measurement and does not overwrite the gate width.
8. Use a reference that occupies a substantial portion of the image. If the calibration conditioning warning appears, the provisional result must not be used; reposition the reference and repeat calibration.

No real-device iPhone test is implied by passing automated geometry tests.
