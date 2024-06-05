import rosys

from field_friend.automations import Plant, PlantProvider


def test_extracting_relevant_crops():
    plants = PlantProvider()
    for i in range(20):
        plants.add_crop(create_crop(i/10.0, 0))
    crops = plants.get_relevant_crops(rosys.geometry.Point(x=1.0, y=0), max_distance=0.45)
    assert len(crops) == 9
    # TODO do not clear list; better to use weighted average in confidence property
    plants.crops[10].confidences.clear()
    plants.crops[10].confidences.append(0.4)
    crops = plants.get_relevant_crops(rosys.geometry.Point(x=1.0, y=0), max_distance=0.45)
    assert len(crops) == 8, 'crops with less than 0.5 confidence should be ignored'
    del plants.crops[9].positions[0]
    crops = plants.get_relevant_crops(rosys.geometry.Point(x=1.0, y=0), max_distance=0.45)
    assert len(crops) == 7, 'crops with less than 3 positions should be ignored'


def create_crop(x: float, y: float) -> Plant:
    """Creates a maize plant with three observed positions at the given coordinates."""
    plant = Plant(type='maize', detection_time=rosys.time())
    for _ in range(3):
        plant.positions.append(rosys.geometry.Point(x=x, y=y))
        plant.confidences.append(0.9)
    return plant
