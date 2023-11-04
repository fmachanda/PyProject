import cv2
import imutils
import matplotlib.pyplot as plt
import numpy as np

def process(image_path: str, radius: int = 160) -> None:
    """Find contours in image for landing UAV."""
    image = cv2.imread(image_path)
    if image is None:
        return
    image = imutils.resize(image, 1024, 1024)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    image = cv2.GaussianBlur(image, (3, 3), 1.0)

    yc, xc = image.shape[:2]
    
    cmask = np.zeros(image.shape[:2], dtype='uint8')
    cmask = cv2.circle(cmask, (xc // 2,yc // 2), radius, (1), -1)

    inner = cv2.bitwise_and(image, image, mask=cmask)
    cmask = 1 - cmask
    outer = cv2.bitwise_and(image, image, mask=cmask)

    thresh = [None, None]

    for n, im in enumerate([inner, outer]):
        final = cv2.normalize(im, None, 0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)

        lowerb = np.min(np.min(final, axis=0), axis=0)
        upperb = np.max(np.max(final, axis=0), axis=0)
        upperb[2] *= 0.3

        mask = cv2.inRange(final, lowerb, upperb)

        final = cv2.bitwise_and(final, final, mask=mask)

        final = cv2.cvtColor(final, cv2.COLOR_RGB2GRAY)

        _, thresh[n] = cv2.threshold(final, 0.3, 1.0, cv2.THRESH_BINARY)
    
    final = cv2.add(thresh[0], thresh[1])
    final = np.array(final, np.uint8)

    contours, _ = cv2.findContours(final, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contoured = np.zeros_like(image)
    cv2.drawContours(contoured, contours, -1, (0,255,0))

    plt.figure()
    plt.imshow(image)
    plt.show()
    plt.imshow(contoured)
    plt.show()


if __name__ == '__main__':
    process('./image1.png')
    # for i in range(1,10):
    #     process(f'./image{i}.png')