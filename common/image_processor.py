import cv2
import cv2.typing
import imutils
import matplotlib.pyplot as plt
import numpy as np

templates = np.load('./common/templates.npy')


def process(image: str | cv2.typing.MatLike, radius: int = 160) -> tuple[np.ndarray]:
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

    return contours, final, image


def find(image: str | cv2.typing.MatLike, radius: int = 160, display: bool = False) -> tuple[int]:
    contours, final, image = process(image)
    strengths = np.empty((len(contours)))

    for n, contour in enumerate(contours):
        x, y, w, h = cv2.boundingRect(contour)
        roi = final[y:y+h, x:x+w]

        if roi.shape[0] < 20 and roi.shape[1] < 20:
            continue

        roi = cv2.resize(roi, (224, 224))

        strength = np.zeros((18))
        negstrength = np.zeros((18))

        for i, template in enumerate(templates):


            negtemplate = 1 - template
            negroi = 1 - roi

            posmatch = np.multiply(template, roi)
            negmatch = np.multiply(negtemplate, negroi)

            match = (np.sum(negroi)*posmatch + np.sum(roi)*negmatch) / (np.sum(negroi) + np.sum(roi))


            strength[i] = 2 * np.sum(match) / (224*224)

        strengths[n] = np.max(strength)


    index = np.where(strengths==np.max(strengths))[0][0]

    x, y, w, h = cv2.boundingRect(contours[index])

    yc = image.shape[0]//2 - (y + h//2)
    xc = (x + w//2) - image.shape[1]//2

    if display:
        cv2.rectangle(image, (x, y), (x+w, y+h), color=(200,0,200), thickness=2)

        # cv2.imshow("Result (Ctrl-W to close)", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        # if cv2.waitKey(0):
        #     cv2.destroyAllWindows()

        plt.figure()
        plt.imshow(image)
        plt.show()
        
    return xc, yc


if __name__ == '__main__':
    print(find('./stored_images/noised.png', display=True))
