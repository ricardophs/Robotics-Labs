# https://photoscissors.com/upload

# Cinzento:
# rgba(241,243,244,255)
# Branco:
# rgba(255,255,255,255)
# Background:
# rgba(229,227,223,255)
# Boarder1:
# rgba(222,223,225,255)
# Boarder2:
# rgba(221,223,227,255)

import cv2
import numpy as np

original_image = cv2.imread("map.png")
t = 3

road_thresh = cv2.inRange(original_image, np.array([241 - t, 243 - t, 244 - t]), np.array([241 + t, 243 + t, 244 + t]))

combined_mask = road_thresh
combined_mask_inv = 255 - combined_mask

combined_mask_rgb = cv2.cvtColor(combined_mask_inv, cv2.COLOR_GRAY2BGR)

final = cv2.max(original_image, combined_mask_rgb)

cv2.imshow('Output', combined_mask_rgb)
cv2.waitKey(0)