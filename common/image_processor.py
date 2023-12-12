import asyncio
import cv2
import cv2.typing
# import logging
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import time

os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/..')
sys.path.append(os.getcwd())
templates: np.ndarray = np.load('./common/train_data.npy')
invtemplates = 1 - templates

VECTORIZE = False

CONFIDENCE_THRESHOLD = 0.3
ROI_MIN_WIDTH = 15
ROI_MIN_HEIGHT = 15
ANNOTATION_COLOR = (200, 0, 200)
ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT = templates.shape[1:]

last_time = 0

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
    global last_time
    now = time.perf_counter_ns()
    print(f"[IMG] Starting find_contour   {((now - last_time)/1e3):.2f}")
    last_time = now
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

    processed = cv2.cvtColor(processed, cv2.COLOR_RGB2HSV)
    lowerb = np.array([40, 0, 0], dtype=np.uint8)
    upperb = np.array([60, 1.0, 1.0], dtype=np.uint8)
    mask = cv2.inRange(processed, lowerb, upperb)
    processed = cv2.bitwise_and(processed, processed, mask=mask)
    processed = cv2.cvtColor(processed, cv2.COLOR_HSV2RGB)
    processed = cv2.cvtColor(processed, cv2.COLOR_RGB2GRAY)
    processed *= 255
    processed = np.array(processed, np.uint8)
    await asyncio.sleep(0)

    _, processed = cv2.threshold(processed, 60, 1.0, cv2.THRESH_BINARY)
    await asyncio.sleep(0)

    contours, _ = cv2.findContours(processed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = [contour for contour in contours if (rect:=cv2.boundingRect(contour))[2] > ROI_MIN_WIDTH and rect[3] > ROI_MIN_HEIGHT]
    await asyncio.sleep(0)

    # contoured = np.zeros_like(processed)
    # cv2.drawContours(contoured, contours, -1, ANNOTATION_COLOR)
    # plt.figure()
    # plt.imshow(contoured)
    # plt.show()

    print(f"[IMG] {len(contours)} contours detected")
    now = time.perf_counter_ns()
    print(f"[IMG] Ending find_contour     {((now - last_time)/1e3):.2f}")
    last_time = now
    return contours, processed, image


async def find_h(image: str | cv2.typing.MatLike, display: bool = False, confidence_threshold: float = CONFIDENCE_THRESHOLD) -> tuple | bool:
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
    global last_time
    now = time.perf_counter_ns()
    print(f"[IMG] Starting find_h...")
    last_time = now

    await asyncio.sleep(0)

    now = time.perf_counter_ns()
    print(f"[IMG] Calling find_contour    {((now - last_time)/1e3):.2f}")
    last_time = now
    if out := await find_contour(image):
        contours, processed, image = out
    else:
        print(f"[IMG] Cannot read image!")
        return False

    if not contours:
        return False
    await asyncio.sleep(0)

    strengths = np.zeros((len(contours), len(templates)))
    await asyncio.sleep(0)

    if VECTORIZE:
        global invtemplates
        print(f"[IMG] Starting numpy loop...")
        s_time = time.perf_counter_ns()
        rois = [cv2.resize(cv2.drawContours(np.zeros_like(processed), contours, n, (1.0), thickness=cv2.FILLED)[(r:=cv2.boundingRect(contours[n]))[1]:r[1]+r[3], r[0]:r[0]+r[2]], (ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT)) for n in range(len(contours))]
        rois = np.stack(rois, axis=0)
        print(f"[IMG] Loop time: ************ {((time.perf_counter_ns() - s_time)/1e6):.2f}")

        posmatches = templates * rois[:, np.newaxis, :, :]
        negmatches = invtemplates * rois[:, np.newaxis, :, :]

        sum_posmatches = np.sum(posmatches, axis=(2, 3))
        sum_negmatches = np.sum(negmatches, axis=(2, 3))

        sum_templates = np.sum(templates, axis=(1, 2))
        sum_templates = sum_templates[np.newaxis, :]

        strengths = (sum_posmatches - sum_negmatches) / sum_templates
    else:
        print(f"[IMG] Starting 'for' loop...")
        t_time = 0.0
        for n, contour in enumerate(contours):
            s_time = time.perf_counter_ns()
            x, y, w, h = cv2.boundingRect(contour)
            if not ((1/3) < (w/h) < (3/1)):
                continue
            # await asyncio.sleep(0)

            roi = np.zeros_like(processed)
            roi = cv2.drawContours(roi, [contour], 0, (1.0), thickness=cv2.FILLED)

            roi: np.ndarray = roi[y:y+h, x:x+w]
            # await asyncio.sleep(0)

            t_time += time.perf_counter_ns() - s_time

            roi = cv2.resize(roi, (ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT))
            # await asyncio.sleep(0)

            invtemplates = 1 - templates

            posmatches = np.multiply(templates, roi)
            negmatches = np.multiply(invtemplates, roi)
            # await asyncio.sleep(0)

            matches = posmatches - negmatches

            strengths[n] = np.sum(matches, axis=(1,2)) / np.sum(templates, axis=(1,2))
            # await asyncio.sleep(0)
        print(f"[IMG] Loop time: ************ {(t_time/1e6):.2f}")
    
    confidence = np.max(strengths)
    await asyncio.sleep(0)

    if confidence <= 0.0:
        now = time.perf_counter_ns()
        print(f"[IMG] Ending find_h           {((now - last_time)/1e3):.2f}")
        last_time = now
        return False

    if confidence < confidence_threshold:
        now = time.perf_counter_ns()
        print(f"[IMG] Ending find_h           {((now - last_time)/1e3):.2f}")
        last_time = now
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

    now = time.perf_counter_ns()
    print(f"[IMG] Ending find_h           {((now - last_time)/1e3):.2f}")
    last_time = now
    return xc, yc, confidence, image


async def _test(file):
    if out := await find_h(file, display=False):
        dx, dy, confidence, image = out
        print(f"'H' detected in {file} at ({dx},{dy}) with a confidence of {confidence:.2f}.")
        # await grapher.imshow(image)
    else:
        print(f"None detected in {file}.")


if __name__ == '__main__':
    print(f"[IMG] Starting...")
    start = time.perf_counter_ns()
    for i in range(1,4):
        asyncio.run(_test(f'./common/test_images/image{i}.png'))

    # for i in range(1,10):
    #     asyncio.run(_test(f'./common/test_images/test{i}.png'))

    # asyncio.run(_test(f'./common/test_images/test4.png'))
    print(f"[IMG] TOTAL TIME        {((time.perf_counter_ns() - start)/1e6):.2f} ms")