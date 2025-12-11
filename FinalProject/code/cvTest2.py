import cv2
import numpy as np

# Load image
img = cv2.imread("code/blocks.png")
if img is None:
    raise FileNotFoundError("Could not load image.")

orig = img.copy()
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# ------------------------------------
# HSV COLOR RANGES FOR YOUR IMAGE
# ------------------------------------

yellow_lower = np.array([20, 80, 80])
yellow_upper = np.array([35, 255, 255])

blue_lower = np.array([95, 80, 80])
blue_upper = np.array([130, 255, 255])

red_lower1 = np.array([0, 80, 80])
red_upper1 = np.array([10, 255, 255])
red_lower2 = np.array([170, 80, 80])
red_upper2 = np.array([180, 255, 255])

# ------------------------------------
# Blob Detector
# ------------------------------------
def detect_blob(mask, color_name):
    num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    blobs = []
    for i in range(1, num_labels):
        x, y, w, h, area = stats[i]
        if area < 3000:
            continue
        cx, cy = centroids[i]
        blobs.append((int(cx), int(cy), x, y, w, h, color_name))
    return blobs

mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
mask_blue   = cv2.inRange(hsv, blue_lower, blue_upper)
mask_red    = cv2.bitwise_or(
                cv2.inRange(hsv, red_lower1, red_upper1),
                cv2.inRange(hsv, red_lower2, red_upper2)
              )

blobs = []
blobs += detect_blob(mask_yellow, "Yellow")
blobs += detect_blob(mask_blue,   "Blue")
blobs += detect_blob(mask_red,    "Red")

# ------------------------------------
# Detect Center Circle
# ------------------------------------
gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)

circles = cv2.HoughCircles(
    gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=40,
    param1=100, param2=15, minRadius=5, maxRadius=30
)

if circles is None:
    raise RuntimeError("Could not detect center circle!")

circles = np.uint16(np.around(circles))
cx_center, cy_center, r = circles[0][0]

cv2.circle(img, (cx_center, cy_center), r, (0,0,0), 2)
cv2.putText(img, "Center", (cx_center + 10, cy_center),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

# ------------------------------------
# FIXED POSITIONS YOU PROVIDED
# ------------------------------------
positions = {
    "Yellow": (-0.0225,  0.0225),
    "Blue":   ( 0.0225,  0.0225),
    "Red":    ( 0.0000, -0.0225)
}

# ------------------------------------
# LABEL EACH BLOCK ACCORDING TO COLOR
# ------------------------------------
for (cx, cy, x, y, w, h, color_name) in blobs:
    dx, dy = positions[color_name]

    label = f"Target - {color_name} - ({dx:.4f}, {dy:.4f}) m"

    cv2.rectangle(img, (x,y), (x+w, y+h), (0,0,0), 2)
    cv2.circle(img, (cx, cy), 6, (0,0,0), -1)
    cv2.putText(img, label, (cx - 90, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 2)

# ------------------------------------
# Save Result
# ------------------------------------
cv2.imwrite("final_output_fixed_positions.png", img)
print("Saved final_output_fixed_positions.png")
