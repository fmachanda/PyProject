import asyncio
import cv2
import cv2.typing
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import threading
import time

os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/..')
sys.path.append(os.getcwd())
templates: np.ndarray = np.load('./common/train_data_v3.npy')
invtemplates = 1 - templates

VECTORIZE = True

CONFIDENCE_THRESHOLD = 0.7
ROI_MIN_DIM = 25
ANNOTATION_COLOR = (200, 0, 200)
ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT = templates.shape[1:]

last_time = 0
threaded_output = False

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
    # print(f"[IMG] Starting find_contour   {((now - last_time)/1e3):.2f}")
    last_time = now
    await asyncio.sleep(0)
    
    if isinstance(image, str):
        image = cv2.imread(image)
        if image is None:
            return False
        
    await asyncio.sleep(0)

    image = cv2.resize(image, (1024, int(image.shape[0] * (1024/image.shape[1]))))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = cv2.GaussianBlur(image, (9, 9), 0)
    await asyncio.sleep(0)

    normed = cv2.normalize(image, None, 0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    await asyncio.sleep(0)

    sobel_x = cv2.Sobel(normed, cv2.CV_32F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(normed, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
    magnitude = cv2.normalize(magnitude, None, 0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    await asyncio.sleep(0)

    # filtered = cv2.cvtColor(normed, cv2.COLOR_RGB2HSV)
    # lowerb = np.array([40, 0, 0], dtype=np.uint8)
    # upperb = np.array([60, 1.0, 1.0], dtype=np.uint8)
    # mask = cv2.inRange(filtered, lowerb, upperb)
    # filtered = cv2.bitwise_and(filtered, filtered, mask=mask)
    # filtered = cv2.cvtColor(filtered, cv2.COLOR_HSV2RGB)
    # filtered = cv2.cvtColor(filtered, cv2.COLOR_RGB2GRAY)

    processed = cv2.cvtColor(magnitude, cv2.COLOR_RGB2GRAY)
    _, processed = cv2.threshold(processed, 0.25, 1.0, cv2.THRESH_BINARY)
    await asyncio.sleep(0)
    processed = cv2.GaussianBlur(processed, (5,5), 0)
    _, processed = cv2.threshold(processed, 0.25, 1.0, cv2.THRESH_BINARY)
    await asyncio.sleep(0)

    contours, _ = cv2.findContours(processed.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    await asyncio.sleep(0)
    contours = [contour for contour in contours if min((rect:=cv2.boundingRect(contour))[2], rect[3]) >= ROI_MIN_DIM]
    await asyncio.sleep(0)

    # contoured = np.zeros_like(processed)
    # cv2.drawContours(contoured, contours, -1, ANNOTATION_COLOR)
    # # plt.figure()
    # plt.imshow(image)
    # plt.show()
    # # plt.imshow(normed)
    # # plt.show()
    # plt.imshow(magnitude, cmap='gray')
    # plt.show()
    # plt.imshow(processed)
    # plt.show()
    # plt.imshow(contoured)
    # plt.show()

    # print(f"[IMG] {len(contours)} contours detected")
    now = time.perf_counter_ns()
    # print(f"[IMG] Ending find_contour     {((now - last_time)/1e3):.2f}")
    last_time = now
    return contours, processed, image


async def find_h(img: str | cv2.typing.MatLike, display: bool = False, confidence_threshold: float = CONFIDENCE_THRESHOLD) -> tuple | bool:
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

    global invtemplates
    global threaded_output

    global last_time
    now = time.perf_counter_ns()
    # print(f"[IMG] Starting find_h...")
    last_time = now

    await asyncio.sleep(0)

    now = time.perf_counter_ns()
    # print(f"[IMG] Calling find_contour    {((now - last_time)/1e3):.2f}")
    last_time = now
    if out := await find_contour(img):
        contours, processed, image = out
    else:
        # print(f"[IMG] Cannot read image!")
        return False

    if not contours:
        return False
    await asyncio.sleep(0)

    strengths = np.zeros((len(contours), len(templates)))
    await asyncio.sleep(0)

    if VECTORIZE:
        # print(f"[IMG] Starting numpy loop...")
        s_time = time.perf_counter_ns()
        rois = [
            cv2.resize(
                cv2.drawContours(
                    np.zeros_like(processed), 
                    [contour], 
                    0, 
                    (1.0), 
                    thickness=cv2.FILLED
                )[(r:=cv2.boundingRect(contour))[1]:r[1]+r[3], r[0]:r[0]+r[2]], 
                (ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT)
            ) for contour in contours
        ]
        await asyncio.sleep(0)
        rois = np.stack(rois, axis=0)
        # print(f"[IMG] Loop time: ************ {((time.perf_counter_ns() - s_time)/1e3):.2f}")
        await asyncio.sleep(0)

        posmatches = templates * rois[:, np.newaxis, :, :]
        await asyncio.sleep(0)
        negmatches = invtemplates * rois[:, np.newaxis, :, :]
        await asyncio.sleep(0)
        posinvmatches = templates * (1-rois)[:, np.newaxis, :, :]
        await asyncio.sleep(0)

        sum_templates = np.sum(templates, axis=(1, 2))
        await asyncio.sleep(0)
        sum_templates = sum_templates[np.newaxis, :]
        await asyncio.sleep(0)

        sum_roi = np.sum(rois, axis=(1, 2))[:, np.newaxis]
        await asyncio.sleep(0)
        
        sum_posmatches = np.sum(posmatches, axis=(2, 3))
        await asyncio.sleep(0)
        sum_negmatches = np.sum(negmatches, axis=(2, 3))
        await asyncio.sleep(0)
        sum_posinvmatches = np.sum(posinvmatches, axis=(2, 3))
        await asyncio.sleep(0)

        strengths = ((templates.shape[1]*templates.shape[2])*(sum_posmatches - sum_negmatches - sum_posinvmatches)) / (sum_templates*sum_roi)
        await asyncio.sleep(0)
    else:
        # print(f"[IMG] Starting 'for' loop...")
        t_time = 0.0
        for n, contour in enumerate(contours):
            s_time = time.perf_counter_ns()
            x, y, w, h = cv2.boundingRect(contour)
            if not ((1/3) < (w/h) < (3/1)):
                continue
            await asyncio.sleep(0)

            roi = np.zeros_like(processed)
            roi = cv2.drawContours(roi, [contour], 0, (1.0), thickness=cv2.FILLED)

            roi: np.ndarray = roi[y:y+h, x:x+w]
            await asyncio.sleep(0)


            roi = cv2.resize(roi, (ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT))
            t_time += time.perf_counter_ns() - s_time
            await asyncio.sleep(0)

            posmatches = np.multiply(templates, roi) # i should be defined and i am defined
            negmatches = np.multiply(invtemplates, roi) # i shouldnt be defined but i am
            posinvmatches = np.multiply(templates, 1-roi) # i should be defined but im not
            # if True:
            #     print(f"**** {n} ****")
            #     plt.figure()
            #     plt.title("ROI")
            #     plt.imshow(roi)
            #     plt.show()
            #     for i in range(18, 36): #range(27, 30):
            #         # print(np.sum(templates[i]))
            #         # plt.imshow(templates[i])
            #         # plt.show()
            #         # print(np.sum(invtemplates[i]))
            #         # plt.imshow(invtemplates[i])
            #         # plt.show()
            #         print(f"---- {i} ----")
            #         print(np.sum(posmatches[i]/np.sum(templates[i])))
            #         # plt.title("POS +")
            #         # plt.imshow(posmatches[i]/np.sum(templates[i]))
            #         # plt.show()

            #         print(np.sum(negmatches[i]/np.sum(templates[i])))
            #         # plt.title("NEG -")
            #         # plt.imshow(negmatches[i]/np.sum(templates[i]))
            #         # plt.show()

            #         print(np.sum(posinvmatches[i]/np.sum(templates[i])))
            #         # plt.title("POSINV -")
            #         # plt.imshow(posinvmatches[i]/np.sum(templates[i]))
            #         # plt.show()

            #         # print(np.sum(neginvmatches[i]/np.sum(invtemplates[i])))
            #         # plt.title("NEGINV +")
            #         # plt.imshow(neginvmatches[i]/np.sum(invtemplates[i]))
            #         # plt.show()

            #         print((templates.shape[1]*templates.shape[2]) * np.sum(posmatches[i]/np.sum(templates[i]) - negmatches[i]/np.sum(templates[i]) - posinvmatches[i]/np.sum(templates[i])) / np.sum(roi))
            #         plt.title("ALL")
            #         plt.imshow(posmatches[i]/np.sum(templates[i]) - negmatches[i]/np.sum(templates[i]) - posinvmatches[i]/np.sum(templates[i]))
            #         plt.show()
            
            await asyncio.sleep(0)

            posstrength = np.sum(posmatches, axis=(1,2)) / np.sum(templates, axis=(1,2))
            negstrength = np.sum(negmatches, axis=(1,2)) / np.sum(templates, axis=(1,2))
            posinvstrength = np.sum(posinvmatches, axis=(1,2)) / np.sum(templates, axis=(1,2))
            await asyncio.sleep(0)

            strengths[n] = (templates.shape[1]*templates.shape[2]) * (posstrength - negstrength - posinvstrength) / np.sum(roi)
            await asyncio.sleep(0)
        # print(f"[IMG] Loop time: ************ {(t_time/1e3):.2f}")
    
    confidence = np.max(strengths)
    await asyncio.sleep(0)

    if confidence < confidence_threshold:
        now = time.perf_counter_ns()
        # print(f"[IMG] Ending find_h           {((now - last_time)/1e3):.2f}")
        last_time = now
        return False

    index = np.where(strengths==confidence)
    await asyncio.sleep(0)
    
    x, y, w, h = cv2.boundingRect(contours[index[0][0]])
    yc = image.shape[0]//2 - (y + h//2)
    xc = (x + w//2) - image.shape[1]//2
    await asyncio.sleep(0)

    cv2.rectangle(image, (x, y), (x+w, y+h), color=ANNOTATION_COLOR, thickness=2)
    cv2.circle(image, (x + w//2, y + h//2), radius=2, color=ANNOTATION_COLOR, thickness=-1)
    await asyncio.sleep(0)

    if __name__=='__main__':
        print(f"'H' detected in {img} at ({xc},{yc}) with a confidence of {confidence:.2f}.")
    if display:
        plt.figure()
        if VECTORIZE:
            plt.imshow(rois[index[0][0]])
            # np.save('./common/output_data.npy', rois[index[0][0]])
            plt.show()
        plt.imshow(image)
        plt.show()
    await asyncio.sleep(0)

    now = time.perf_counter_ns()
    # print(f"[IMG] Ending find_h           {((now - last_time)/1e3):.2f}")
    last_time = now
    threaded_output = (xc, yc, confidence, image)
    return xc, yc, confidence, image


def sync_proc(img: str | cv2.typing.MatLike, display: bool = True, confidence_threshold: float = CONFIDENCE_THRESHOLD) -> tuple | bool:
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
    # print(f"[IMG] Starting find_h...")
    last_time = now

    
    if isinstance(img, str):
        og = cv2.imread(img)
        if og is None:
            return False

    og = cv2.resize(og, (1024, int(og.shape[0] * (1024/og.shape[1]))))
    image = cv2.cvtColor(og, cv2.COLOR_BGR2RGB)
    image = cv2.GaussianBlur(image, (9, 9), 0)

    normed = cv2.normalize(image, None, 0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)

    sobel_x = cv2.Sobel(normed, cv2.CV_32F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(normed, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
    magnitude = cv2.normalize(magnitude, None, 0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)

    # filtered = cv2.cvtColor(normed, cv2.COLOR_RGB2HSV)
    # lowerb = np.array([40, 0, 0], dtype=np.uint8)
    # upperb = np.array([60, 1.0, 1.0], dtype=np.uint8)
    # mask = cv2.inRange(filtered, lowerb, upperb)
    # filtered = cv2.bitwise_and(filtered, filtered, mask=mask)
    # filtered = cv2.cvtColor(filtered, cv2.COLOR_HSV2RGB)
    # filtered = cv2.cvtColor(filtered, cv2.COLOR_RGB2GRAY)

    processed = cv2.cvtColor(magnitude, cv2.COLOR_RGB2GRAY)
    _, processed = cv2.threshold(processed, 0.25, 1.0, cv2.THRESH_BINARY)
    processed = cv2.GaussianBlur(processed, (5,5), 0)
    _, processed = cv2.threshold(processed, 0.25, 1.0, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(processed.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = [contour for contour in contours if min((rect:=cv2.boundingRect(contour))[2], rect[3]) >= ROI_MIN_DIM]

    # contoured = np.zeros_like(processed)
    # cv2.drawContours(contoured, contours, -1, ANNOTATION_COLOR)
    # # plt.figure()
    # plt.imshow(image)
    # plt.show()
    # # plt.imshow(normed)
    # # plt.show()
    # plt.imshow(magnitude, cmap='gray')
    # plt.show()
    # plt.imshow(processed)
    # plt.show()
    # plt.imshow(contoured)
    # plt.show()

    # print(f"[IMG] {len(contours)} contours detected")
    now = time.perf_counter_ns()
    # print(f"[IMG] Ending find_contour     {((now - last_time)/1e3):.2f}")
    last_time = now

    if not contours:
        return False

    strengths = np.zeros((len(contours), len(templates)))

    if VECTORIZE:
        # print(f"[IMG] Starting numpy loop...")
        s_time = time.perf_counter_ns()
        rois = [
            cv2.resize(
                cv2.drawContours(
                    np.zeros_like(processed), 
                    [contour], 
                    0, 
                    (1.0), 
                    thickness=cv2.FILLED
                )[(r:=cv2.boundingRect(contour))[1]:r[1]+r[3], r[0]:r[0]+r[2]], 
                (ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT)
            ) for contour in contours
        ]
        rois = np.stack(rois, axis=0)
        # print(f"[IMG] Loop time: ************ {((time.perf_counter_ns() - s_time)/1e3):.2f}")

        posmatches = templates * rois[:, np.newaxis, :, :]
        negmatches = invtemplates * rois[:, np.newaxis, :, :]
        posinvmatches = templates * (1-rois)[:, np.newaxis, :, :]

        sum_templates = np.sum(templates, axis=(1, 2))
        sum_templates = sum_templates[np.newaxis, :]

        sum_roi = np.sum(rois, axis=(1, 2))[:, np.newaxis]
        
        sum_posmatches = np.sum(posmatches, axis=(2, 3))
        sum_negmatches = np.sum(negmatches, axis=(2, 3))
        sum_posinvmatches = np.sum(posinvmatches, axis=(2, 3))

        strengths = ((templates.shape[1]*templates.shape[2])*(sum_posmatches - sum_negmatches - sum_posinvmatches)) / (sum_templates*sum_roi)
    else:
        # print(f"[IMG] Starting 'for' loop...")
        t_time = 0.0
        for n, contour in enumerate(contours):
            s_time = time.perf_counter_ns()
            x, y, w, h = cv2.boundingRect(contour)
            if not ((1/3) < (w/h) < (3/1)):
                continue

            roi = np.zeros_like(processed)
            roi = cv2.drawContours(roi, [contour], 0, (1.0), thickness=cv2.FILLED)

            roi: np.ndarray = roi[y:y+h, x:x+w]


            roi = cv2.resize(roi, (ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT))
            t_time += time.perf_counter_ns() - s_time

            posmatches = np.multiply(templates, roi) # i should be defined and i am defined
            negmatches = np.multiply(invtemplates, roi) # i shouldnt be defined but i am
            posinvmatches = np.multiply(templates, 1-roi) # i should be defined but im not
            # if True:
            #     print(f"**** {n} ****")
            #     plt.figure()
            #     plt.title("ROI")
            #     plt.imshow(roi)
            #     plt.show()
            #     for i in range(18, 36): #range(27, 30):
            #         # print(np.sum(templates[i]))
            #         # plt.imshow(templates[i])
            #         # plt.show()
            #         # print(np.sum(invtemplates[i]))
            #         # plt.imshow(invtemplates[i])
            #         # plt.show()
            #         print(f"---- {i} ----")
            #         print(np.sum(posmatches[i]/np.sum(templates[i])))
            #         # plt.title("POS +")
            #         # plt.imshow(posmatches[i]/np.sum(templates[i]))
            #         # plt.show()

            #         print(np.sum(negmatches[i]/np.sum(templates[i])))
            #         # plt.title("NEG -")
            #         # plt.imshow(negmatches[i]/np.sum(templates[i]))
            #         # plt.show()

            #         print(np.sum(posinvmatches[i]/np.sum(templates[i])))
            #         # plt.title("POSINV -")
            #         # plt.imshow(posinvmatches[i]/np.sum(templates[i]))
            #         # plt.show()

            #         # print(np.sum(neginvmatches[i]/np.sum(invtemplates[i])))
            #         # plt.title("NEGINV +")
            #         # plt.imshow(neginvmatches[i]/np.sum(invtemplates[i]))
            #         # plt.show()

            #         print((templates.shape[1]*templates.shape[2]) * np.sum(posmatches[i]/np.sum(templates[i]) - negmatches[i]/np.sum(templates[i]) - posinvmatches[i]/np.sum(templates[i])) / np.sum(roi))
            #         plt.title("ALL")
            #         plt.imshow(posmatches[i]/np.sum(templates[i]) - negmatches[i]/np.sum(templates[i]) - posinvmatches[i]/np.sum(templates[i]))
            #         plt.show()

            posstrength = np.sum(posmatches, axis=(1,2)) / np.sum(templates, axis=(1,2))
            negstrength = np.sum(negmatches, axis=(1,2)) / np.sum(templates, axis=(1,2))
            posinvstrength = np.sum(posinvmatches, axis=(1,2)) / np.sum(templates, axis=(1,2))

            strengths[n] = (templates.shape[1]*templates.shape[2]) * (posstrength - negstrength - posinvstrength) / np.sum(roi)
        # print(f"[IMG] Loop time: ************ {(t_time/1e3):.2f}")
    
    confidence = np.max(strengths)

    if confidence < confidence_threshold:
        now = time.perf_counter_ns()
        # print(f"[IMG] Ending find_h           {((now - last_time)/1e3):.2f}")
        last_time = now
        return False

    index = np.where(strengths==confidence)
    
    x, y, w, h = cv2.boundingRect(contours[index[0][0]])
    yc = image.shape[0]//2 - (y + h//2)
    xc = (x + w//2) - image.shape[1]//2

    cv2.rectangle(og, (x, y), (x+w, y+h), color=ANNOTATION_COLOR, thickness=2)
    cv2.circle(og, (x + w//2, y + h//2), radius=2, color=ANNOTATION_COLOR, thickness=-1)

    if __name__=='__main__':
        print(f"'H' detected in {img} at ({xc},{yc}) with a confidence of {confidence:.2f}.")
    if display:
        # plt.figure()
        # if VECTORIZE:
        #     plt.imshow(rois[index[0][0]])
        #     # np.save('./common/output_data.npy', rois[index[0][0]])
        #     plt.show()
        # plt.imshow(image)
        # plt.show()
        cv2.imshow("Detected Landing Zone", og)
        cv2.setWindowProperty("Detected Landing Zone", cv2.WND_PROP_TOPMOST, 1)
        if cv2.waitKey(800):
            cv2.destroyAllWindows()

    now = time.perf_counter_ns()
    # print(f"[IMG] Ending find_h           {((now - last_time)/1e3):.2f}")
    last_time = now
    threaded_output = (xc, yc, confidence, image)
    return xc, yc, confidence, image


async def _test(file):
    if out := await find_h(file, display=True):
        dx, dy, confidence, image = out
    else:
        print(f"None detected in {file}.")


if __name__ == '__main__':
    # print(f"[IMG] Starting...")
    start = time.perf_counter_ns()
    # for i in range(1,4):
    #     asyncio.run(_test(f'./common/test_images/image{i}.png'))

    # for i in range(1,9):
    #     asyncio.run(_test(f'./common/test_images/testing{i}.png'))

    # import os
    # directory_path = './common/test_images'
    # for filename in os.listdir(directory_path):
    #     file_path = os.path.join(directory_path, filename)
    #     if os.path.isfile(file_path):
    #         asyncio.run(_test(file_path))

    names = []
    # names.append('Cessna_172SP - 2023-11-05 09.01.09')

    # names.append('Cessna_172SP - 2023-11-05 10.36.57')
    # names.append('Cessna_172SP - 2023-11-05 10.38.41')
    # names.append('Cessna_172SP - 2023-11-05 10.51.28') #*
    # names.append('Cessna_172SP - 2023-11-05 10.54.38') #*  n=3
    # names.append('Cessna_172SP - 2023-11-05 10.56.34') #* n=2
    # names.append('image4')
    # names.append('image9')
    # names.append('noised2')
    # names.append('testing7') #**

    # names.append('Cessna_172SP - 2023-11-05 10.37.37')
    # names.append('Cessna_172SP - 2023-11-05 10.37.45')
    # names.append('Cessna_172SP - 2023-11-05 10.37.52')
    # names.append('Cessna_172SP - 2023-11-05 10.38.00')

    # names.append('Cessna_172SP - 2023-11-05 08.57.50')
    names.append('fmuas - 2023-12-06 02.28.57')

    for name in names:
        asyncio.run(_test(f'./common/test_images/{name}.png'))

    # print(f"[IMG] TOTAL TIME        {((time.perf_counter_ns() - start)/1e6):.2f} ms")
