import cv2
import numpy as np
import imutils
import matplotlib.pyplot as plt
# import tensorflow as tf

from image_processor import process

file = './stored_images/image2.png'

cnts = [20, 11, 14, 11, 10, 11, 9, 6, 7, 5, 9, 10, 11, 13, 13, 14, 11, 11, 40]

master = np.zeros((18, 224, 224))

def rotate_image(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  return result

def preprocess_image(img):
    img = cv2.resize(img, (224, 224))  # Resize to the input size expected by MobileNetV2
    # img = -tf.keras.applications.mobilenet_v2.preprocess_input(img)
    return img

for i in range(18):
    img = cv2.imread(file)
    img = imutils.resize(img, 1024, 1024)
    img = rotate_image(img, 10*i)
    # cv2.imshow("Rotated", img)
    # if cv2.waitKey(0):
    #     cv2.destroyAllWindows()


    final, _, yellow_contours, image = process(img)

    # print(final.shape)
    # print(image.shape)

    print(f'{len(yellow_contours)} contours detected')

    # Assuming 'yellow_contours' contains the detected yellow markings
    for n, contour in enumerate(yellow_contours):
        x, y, w, h = cv2.boundingRect(contour)
        roi = final[y:y+h, x:x+w]

        if roi.shape[0] < 20 or roi.shape[1] < 20:
            continue


        roi = preprocess_image(roi)
        # roi = np.expand_dims(roi, axis=0)  # Add a batch dimension
        # prediction: np.ndarray = model.predict(roi)

        # print(prediction[0][1])

        # new = np.zeros_like(image)
        # cv2.drawContours(new, [contour], -1, (255-i*6, i*6, 255))  # Draw a green bounding box around "H" shape

        # try:
        #     cv2.imshow(f'ROI {i}', roi)
        # except cv2.error:
        #     continue
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        if n == cnts[i]:

            # plt.figure()
            # plt.imshow(roi)
            # plt.show()

            master[0] = roi
            print(roi)
            # print(master)

            # if not cv2.imwrite(f'./templates/H{i}.jpg', roi):
            #     raise Exception("Could not write image")

np.save('./templates/master.npy', master)