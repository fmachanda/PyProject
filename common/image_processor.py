import cv2
import cv2.typing
import imutils
import matplotlib.pyplot as plt
import numpy as np

CONFIDENCE_THRESHOLD = 0.50

ROI_MIN_WIDTH = 30
ROI_MIN_HEIGHT = 30

ANNOTATION_COLOR = (200, 0, 200)

templates = np.load('./common/templates.npy')

ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT = templates.shape[1:]

def find_contour(image: str | cv2.typing.MatLike, radius: int = 160) -> tuple[np.ndarray] | bool:
    """Find contours in image for landing UAV.
    
    Parameters
    ----------
    image : str | cv2.typing.MatLike
        Image or image path.
    radius : int
        Mask radius to account for landing light.

    Returns
    -------
    tuple[np.ndarray, np.ndarray, np.ndarray]
        contours, final, original of input image.
    bool
        False if image doesn't load.
    """
    
    if isinstance(image, str):
        image = cv2.imread(image)
        if image is None:
            return False

    image = imutils.resize(image, 1024)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    image = cv2.GaussianBlur(image, (3, 3), 0)

    yc, xc = image.shape[:2]
    
    cmask = np.zeros(image.shape[:2], dtype='uint8')
    cmask = cv2.circle(cmask, (xc // 2,yc // 2), radius, (1), -1)

    inner = cv2.bitwise_and(image, image, mask=cmask)
    cmask = 1 - cmask
    outer = cv2.bitwise_and(image, image, mask=cmask)

    out = []

    for im in [inner, outer]:
        working = cv2.normalize(im, None, 0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)

        lowerb = np.min(np.min(working, axis=0), axis=0)
        upperb = np.max(np.max(working, axis=0), axis=0)
        upperb[2] *= 0.3
        mask = cv2.inRange(working, lowerb, upperb)

        working = cv2.bitwise_and(working, working, mask=mask)
        working = cv2.cvtColor(working, cv2.COLOR_RGB2GRAY)
        working *= 255
        working = np.array(working, dtype=np.uint8)
        _, working = cv2.threshold(working, 0.3, 1.0, cv2.THRESH_BINARY)

        out.append(working)
    
    final = cv2.add(out[0], out[1])
    final = np.array(final, np.uint8)

    contours, _ = cv2.findContours(final, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return contours, final, image


def find(image: str | cv2.typing.MatLike, radius: int = 160, display: bool = False, confidence_threshold: float = CONFIDENCE_THRESHOLD) -> tuple | bool:
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

    if out := find_contour(image):
        contours, final, image = out
    else:
        return False
    confidences = np.zeros((len(contours)))

    for n, contour in enumerate(contours):
        x, y, w, h = cv2.boundingRect(contour)
        roi = final[y:y+h, x:x+w]

        if roi.shape[0] < ROI_MIN_WIDTH or roi.shape[1] < ROI_MIN_HEIGHT:
            continue

        ratio = roi.shape[0]/roi.shape[1]
        if not (1/3 < ratio < 3/1):
            continue

        roi = cv2.resize(roi, (ROI_RESCALE_WIDTH, ROI_RESCALE_HEIGHT))

        strength = np.zeros((templates.shape[0]))
        negstrength = np.zeros((templates.shape[0]))

        for i, template in enumerate(templates):
            negtemplate = 1 - template
            negroi = 1 - roi

            posmatch = np.multiply(template, roi)
            negmatch = np.multiply(negtemplate, negroi)

            match = (np.sum(negroi)*posmatch + np.sum(roi)*negmatch) / (np.sum(negroi) + np.sum(roi))
            strength[i] = 2 * np.sum(match) / (ROI_RESCALE_WIDTH*ROI_RESCALE_HEIGHT)

        confidences[n] = np.max(strength)

    confidence = np.max(confidences)

    if confidence < confidence_threshold:
        return False

    index = np.where(confidences==np.max(confidences))[0][0]
    x, y, w, h = cv2.boundingRect(contours[index])
    yc = image.shape[0]//2 - (y + h//2)
    xc = (x + w//2) - image.shape[1]//2

    if display:
        cv2.rectangle(image, (x, y), (x+w, y+h), color=ANNOTATION_COLOR, thickness=2)
        cv2.circle(image, (x + w//2, y + h//2), radius=2, color=ANNOTATION_COLOR, thickness=-1)
        plt.figure()
        plt.imshow(image)
        plt.show()
        
    return xc, yc, confidence


if __name__ == '__main__':
    for i in range(1,10):
        find(f'./stored_images/image{i}.png', display=True)
    # find('./stored_images/doubled.png', display=True)
