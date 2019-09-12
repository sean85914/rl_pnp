'''
  Rotate given image file -90, -45, 0 and 45 degrees and show the result
'''


import cv2

# PUT IMAGE FILE NAME HERE
img = cv2.imread("color_000100.jpg")
(h, w) = img.shape[:2]
center = (w//2, h//2)
scale = 1.0

angle_neg_90 = -90
angle_neg_45 = -45
angle_0      = 0
angle_pos_45 = 45

M1 = cv2.getRotationMatrix2D(center, angle_neg_90, scale)
M2 = cv2.getRotationMatrix2D(center, angle_neg_45, scale)
M3 = cv2.getRotationMatrix2D(center, angle_0, scale)
M4 = cv2.getRotationMatrix2D(center, angle_pos_45, scale)

cv2.imshow("-90", cv2.warpAffine(img, M1, (w, h)))
cv2.imshow("-45", cv2.warpAffine(img, M2, (w, h)))
cv2.imshow("0", cv2.warpAffine(img, M3, (w, h)))
cv2.imshow("45", cv2.warpAffine(img, M4, (w, h)))
cv2.waitKey(0)
