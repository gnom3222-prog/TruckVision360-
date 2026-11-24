# Sensor / Perception Input Interface

## Purpose

TruckVision 360 accepts **already processed detections** from an external perception computer that fuses LiDAR and camera data. This browser application does not perform LiDAR-camera fusion or promise 50 m detection. Sensor selection, calibration, perception models and transport are separate hardware/system-integration work.

The normal driver workflow does not require manually pointing a phone camera at obstacles, marking corners, or entering object dimensions. The former camera obstacle registration UI has been removed from the main workflow. Camera-based gate measurement remains a separate optional measurement utility.

## Browser API

After `sensor-perception.js` and `truckvision.js` load:

- `window.TruckVisionSetPerceptionFrame(frame)` validates and applies a complete frame.
- `window.TruckVisionClearPerceptionFrame()` clears the active perception frame.

A successful call returns `{ok:true, accepted, frameId}`; invalid input returns `{ok:false, errors}`. Applying a new frame invalidates any previously planned route. A frame older than 1500 ms is removed from the active scene and the route is invalidated. The interface currently retains only the latest frame; the external perception system should send complete, refreshed frames.

## Coordinate convention

The current adapter accepts `coordinateFrame: "vehicle-local"` only:

- X: forward from the tractor reference point.
- Y: left of the tractor.
- Heading: radians, counter-clockwise from +X.
- Every frame must include `vehiclePose: {x, y, heading}`: the tractor reference point and heading in the map coordinate system **at the same timestamp as the detections**.
- Object positions and headings are transformed into map coordinates using that timestamped pose, never the browser's later/current pose.

The external localization source must synchronize its pose with the sensor timestamp. If that pose is unavailable or not synchronized, reject the frame rather than applying a guessed transform.

## Example frame

```js
window.TruckVisionSetPerceptionFrame({
  frameId: "perception-000042",
  timestamp: Date.now(), // Unix epoch milliseconds
  coordinateFrame: "vehicle-local",
  vehiclePose: { x: 100.0, y: 25.0, heading: 1.5708 }, // map pose at this exact timestamp
  objects: [{
    id: "track-17",
    type: "pallet",
    pose: { x: 12.4, y: -1.8, heading: 0.12 }, // metres, radians
    dimensions: { length: 1.2, width: 0.8, height: 1.45 }, // metres
    confidence: 0.97,
    source: "fused" // lidar | camera | fused
  }]
});
```

## Validation limits in this prototype

- Frame timestamp: no more than 1500 ms old and no more than 250 ms in the future.
- Maximum 300 objects per frame.
- Unique non-empty object IDs.
- Finite timestamp-synchronized `vehiclePose` in map coordinates.
- Object center within 60 m of the tractor reference point.
- Finite pose; dimensions length 0.05–30 m and width 0.05–15 m.
- Confidence in [0, 1]; source is lidar, camera or fused.
- Only the latest complete frame is retained.

The 60 m validation envelope is a software input limit, **not a sensor performance claim**. A 50 m operational detection target must be validated against selected hardware, mounting, weather, occlusion, object class and test conditions.

## Not selected yet

No ROS 2, UDP, MQTT, WebSocket, USB or vendor SDK transport has been selected or implemented. This API is the in-browser integration boundary only. The next hardware stage must define message transport, authentication, timestamps/clock synchronization, localization, health monitoring, object tracking and fail-safe behavior for sensor loss.

The application remains a driver-assistance prototype. Its route and clearance calculations are not a certified safety system and do not replace the driver's observation or responsibility.
