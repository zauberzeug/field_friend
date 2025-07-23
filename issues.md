# Advance on spline

```python
def create_segment_simulation(self, segment: WorkingSegment, *, first_plant_distance: float = 0.3, crop_distance: float = 0.3) -> None:
    CROPS = [
        Point3d(x=0.397, y=0.034, z=0),
        Point3d(x=0.742, y=0.117, z=0)
    ]
    for crop_point in CROPS:
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                        position=Point3d(x=crop_point.x, y=crop_point.y, z=0)))
    WEEDS = [
        Point3d(x=0.429, y=-0.052, z=0),
        Point3d(x=0.345, y=0.048, z=0),
        Point3d(x=0.389, y=0.174, z=0),
        Point3d(x=0.375, y=0.051, z=0),
        Point3d(x=0.389, y=0.023, z=0),
        Point3d(x=0.422, y=0.158, z=0),
        Point3d(x=0.766, y=0.041, z=0),
        Point3d(x=0.812, y=-0.008, z=0),
        Point3d(x=0.700, y=0.084, z=0),
        Point3d(x=0.771, y=0.126, z=0),
        Point3d(x=0.684, y=0.173, z=0),
        Point3d(x=0.713, y=0.181, z=0)
    ]
    for weed_point in WEEDS:
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                        position=Point3d(x=weed_point.x, y=weed_point.y, z=0)))
```
