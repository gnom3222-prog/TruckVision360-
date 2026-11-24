# Camera measurement: projective validity and limits

The browser camera workflow uses four manually selected corners of a known rectangular reference and four corners of the opening. It applies a planar homography. It is not a depth sensor, LiDAR scan, or calibrated photogrammetry system.

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

The camera result must not automatically reduce the manually entered clearance reserve. Verify dimensions with a tape measure and use a conservative manually selected reserve.

## iPhone field check

1. Open the HTTPS application in Safari and explicitly tap **Включить камеру**.
2. Deny permission once and confirm that a clear explanation appears and the simulator remains usable.
3. Grant permission, place the reference and opening in the same plane, and do not move the phone.
4. Measure the opening, then repeat after changing the phone angle and distance.
5. Compare every result with a tape measure; record the difference rather than treating the displayed 1σ as total accuracy.
6. Confirm that orientation changes, video-frame dimension changes, stopping the stream, or changing reference dimensions invalidate calibration.
7. Confirm that a projective-horizon error is shown as a failed measurement and does not overwrite the gate width.

No real-device iPhone test is implied by passing automated geometry tests.
