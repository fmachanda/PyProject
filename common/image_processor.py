import cv2
import cv2.typing
import imutils
import numpy as np


def process(image: str | cv2.typing.MatLike, radius: int = 160, display: bool = False) -> np.ndarray:
    if isinstance(image, str):
        image = cv2.imread(image)
        if image is None:
            return

    """Find contours in image for landing UAV."""
    image = imutils.resize(image, 1024, 1024)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    image = cv2.GaussianBlur(image, (3, 3), 0)

    yc, xc = image.shape[:2]
    
    cmask = np.zeros(image.shape[:2], dtype='uint8')
    cmask = cv2.circle(cmask, (xc // 2,yc // 2), radius, (1), -1)

    inner = cv2.bitwise_and(image, image, mask=cmask)
    cmask = 1 - cmask
    outer = cv2.bitwise_and(image, image, mask=cmask)

    out = [None, None]

    for n, im in enumerate([inner, outer]):
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

        out[n] = working
    
    final = cv2.add(out[0], out[1])
    final = np.array(final, np.uint8)

    
    # contoured = cv2.Canny(final, 0, 0)

    contours, _ = cv2.findContours(final, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contoured = np.zeros_like(image)
    # contoured = image.copy()
    cv2.drawContours(contoured, contours, -1, (255,0,255))

    if display:
        cv2.imshow("CV2", contoured)
        if cv2.waitKey(0):
            cv2.destroyAllWindows()

    return final, contoured, contours, image


if __name__ == '__main__':
    process('./stored_images/image1.png', display=True)
