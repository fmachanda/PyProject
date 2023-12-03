import asyncio
import cv2
import cv2.typing
# import logging
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/..')
sys.path.append(os.getcwd())
templates: np.ndarray = np.load('./common/templates.npy')

CONFIDENCE_THRESHOLD = 0.4
ROI_MIN_WIDTH = 15
ROI_MIN_HEIGHT = 15
ANNOTATION_COLOR = (200, 0, 200)
ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT = templates.shape[1:]

async def find_contour(image: str | cv2.typing.MatLike) -> tuple[np.ndarray] | bool:
    """Find contours in image for landing UAV.
    
    Parameters
    ----------
    image : str | cv2.typing.MatLike
        Image or image path.

    Returns
    -------
    tuple[np.ndarray, np.ndarray, np.ndarray]
        contours, processed, original of input image.
    bool
        False if image doesn't load.
    """
    await asyncio.sleep(0)
    
    if isinstance(image, str):
        image = cv2.imread(image)
        if image is None:
            pass # logging.error(f"Cannot read {image}.")
            return False
        
    await asyncio.sleep(0)

    image = cv2.resize(image, (1024, int(image.shape[0] * (1024/image.shape[1]))))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = cv2.GaussianBlur(image, (3, 3), 0)
    await asyncio.sleep(0)

    processed = cv2.normalize(image, None, 0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    await asyncio.sleep(0)

    lowerb = np.min(np.min(processed, axis=0), axis=0)
    upperb = np.max(np.max(processed, axis=0), axis=0)
    upperb[2] *= 0.3
    mask = cv2.inRange(processed, lowerb, upperb)
    processed = cv2.bitwise_and(processed, processed, mask=mask)
    processed = cv2.cvtColor(processed, cv2.COLOR_RGB2GRAY)
    processed *= 255
    processed = np.array(processed, np.uint8)
    await asyncio.sleep(0)

    _, processed = cv2.threshold(processed, 60, 1.0, cv2.THRESH_BINARY)
    await asyncio.sleep(0)

    contours, _ = cv2.findContours(processed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    await asyncio.sleep(0)

    # contoured = np.zeros_like(processed)
    # cv2.drawContours(contoured, contours, -1, ANNOTATION_COLOR)
    # plt.figure()
    # plt.imshow(contoured)
    # plt.show()

    return contours, processed, image


async def find_h(image: str | cv2.typing.MatLike, radius: int = 160, display: bool = False, confidence_threshold: float = CONFIDENCE_THRESHOLD) -> tuple | bool:
    """Find 'H' in image for landing UAV.
    
    Parameters
    ----------
    image : str | cv2.typing.MatLike
        Image or image path.
    radius : int
        Mask radius to account for landing light.
    display : bool
        Should I display any found Hs?
    confidence_threshold : float
        Zero to one of required confidence.

    Returns
    -------
    tuple[int, int, float]
        Hx, Hy, Confidence of calculated image.
    bool
        False if nothing exciting happens.
    """

    await asyncio.sleep(0)

    if out := await find_contour(image):
        contours, processed, image = out
    else:
        return False
    await asyncio.sleep(0)

    strengths = np.zeros((len(contours), len(templates)))
    await asyncio.sleep(0)

    for n, contour in enumerate(contours):
        x, y, w, h = cv2.boundingRect(contour)
        await asyncio.sleep(0)

        roi = np.zeros_like(processed)
        cv2.drawContours(roi, contours, n, (1.0), thickness=cv2.FILLED)
        roi: np.ndarray = roi[y:y+h, x:x+w]
        await asyncio.sleep(0)

        if roi.shape[0] < ROI_MIN_WIDTH or roi.shape[1] < ROI_MIN_HEIGHT:
            continue

        ratio = roi.shape[0]/roi.shape[1]
        if not (1/3 < ratio < 3/1):
            continue

        roi = cv2.resize(roi, (ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT))
        await asyncio.sleep(0)

        invtemplates = 1 - templates

        posmatches = np.multiply(templates, roi)
        negmatches = np.multiply(invtemplates, roi)
        await asyncio.sleep(0)

        matches = posmatches - negmatches

        strengths[n] = np.sum(matches, axis=(1,2)) / np.sum(templates, axis=(1,2))
        await asyncio.sleep(0)
    
    confidence = np.max(strengths)
    await asyncio.sleep(0)

    if confidence <= 0.0:
        return False

    if confidence < confidence_threshold:
        return False

    index = np.where(strengths==confidence)[0][0]
    await asyncio.sleep(0)
    
    x, y, w, h = cv2.boundingRect(contours[index])
    yc = image.shape[0]//2 - (y + h//2)
    xc = (x + w//2) - image.shape[1]//2
    await asyncio.sleep(0)

    cv2.rectangle(image, (x, y), (x+w, y+h), color=ANNOTATION_COLOR, thickness=2)
    cv2.circle(image, (x + w//2, y + h//2), radius=2, color=ANNOTATION_COLOR, thickness=-1)
    await asyncio.sleep(0)

    if display:
        plt.figure()
        plt.imshow(image)
        plt.show()
    await asyncio.sleep(0)

    return xc, yc, confidence, image


async def _test(file):
    if out := await find_h(file, display=True):
        x_offset, y_offset, confidence, _ = out
        print(f"'H' detected in {file} at ({x_offset},{y_offset}) with a confidence of {confidence:.2f}.")
    else:
        print(f"None detected in {file}.")


if __name__ == '__main__':
    for i in range(1,4):
        asyncio.run(_test(f'./stored_images/image{i}.png'))

    # _test('./stored_images/noised.png')
