import rosys

from field_friend.automations import Plant, PlantProvider


def test_extracting_relevant_crops():
    plants = PlantProvider()
    for i in range(20):
        plants.add_crop(create_crop(i/10.0, 0))
    crops = plants.get_relevant_crops(rosys.geometry.Point3d(x=1.0, y=0, z=0), max_distance=0.45)
    assert len(crops) == 9
    # TODO do not clear list; better to use weighted average in confidence property
    plants.crops[10].confidences.clear()
    plants.crops[10].confidences.append(0.4)
    crops = plants.get_relevant_crops(rosys.geometry.Point3d(x=1.0, y=0, z=0), max_distance=0.45)
    assert len(crops) == 8, 'crops with a confidence of less than PlantProvider.MINIMUM_COMBINED_CROP_CONFIDENCE should be ignored'


def create_crop(x: float, y: float) -> Plant:
    """Creates a maize plant with three observed positions at the given coordinates."""
    plant = Plant(type='maize', detection_time=rosys.time())
    for _ in range(3):
        plant.positions.append(rosys.geometry.Point3d(x=x, y=y, z=0))
        plant.confidences.append(0.9)
    return plant


def test_crop_prediction():
    plant_provider = PlantProvider()
    plant_provider.predict_crop_position = False
    confidence = 0.4

    def add_crop(x):
        plant = Plant(type='maize', detection_time=rosys.time())
        plant.positions.append(rosys.geometry.Point3d(x=x, y=0, z=0))
        plant.confidences.append(confidence)
        plant_provider.add_crop(plant)

    add_crop(x=0)
    assert plant_provider.crops[0].confidence == confidence
    add_crop(x=plant_provider.crop_spacing)
    assert plant_provider.crops[1].confidence == confidence
    plant_provider.predict_crop_position = True
    add_crop(x=2 * plant_provider.crop_spacing)
    assert plant_provider.crops[2].confidence == (confidence + plant_provider.prediction_confidence)
    plant_provider.predict_crop_position = False
    add_crop(x=3 * plant_provider.crop_spacing)
    assert plant_provider.crops[3].confidence == confidence
