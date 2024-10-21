import random
import time

import cv2
import numpy as np
import PIL
import pytest
import rosys
from rosys.testing import forward
from rosys.vision.image import ImageSize
from rosys.vision.simulated_camera.simulated_device import _floating_text_position


def _create_image_data(id: str, size: ImageSize, color: str) -> bytes:  # pylint: disable=redefined-builtin
    img = PIL.Image.new('RGB', size=(size.width, size.height), color=color)
    d = PIL.ImageDraw.Draw(img)
    text = f'{id}: {time.time():.2f}'
    position = _floating_text_position(size.width, size.height)

    d.text((position.x, position.y), text, fill=(0, 0, 0))
    d.text((position.x + 1, position.y + 1), text, fill=(255, 255, 255))

    _, encoded_image = cv2.imencode('.jpg', np.array(img)[:, :, ::-1])  # NOTE: cv2 expects BGR
    return encoded_image.tobytes()


async def test_cpu_bound_task():
    color = f'#{random.randint(0, 0xffffff):06x}'
    image_size = ImageSize(width=800, height=600)
    for _ in range(10):
        await rosys.run.cpu_bound(_create_image_data, 'test', image_size, color)
        await forward(1)


async def test_task():
    color = f'#{random.randint(0, 0xffffff):06x}'
    image_size = ImageSize(width=800, height=600)
    for _ in range(10):
        _create_image_data('test', image_size, color)
        await forward(1)
