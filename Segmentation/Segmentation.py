import cv2
import numpy as np

def largest_component(mask: np.ndarray) -> np.ndarray:
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return mask
    # Skip label 0 (background). Pick largest by area.
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    out = np.zeros_like(mask)
    out[labels == largest] = 255
    return out

img = cv2.imread("medium_middle.png")
if img is None:
    raise FileNotFoundError("Could not read image")

h, w = img.shape[:2]
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 1. Detect bucket top edge as strongest horizontal edge row
sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
row_energy = np.abs(sobel_y).sum(axis=1)

# Search in a reasonable band (avoid image border + machine arms lower down)
y0 = int(h * 0.15)
y1 = int(h * 0.70)
lip_y = y0 + np.argmax(row_energy[y0:y1])

lip_offset = 100   # + moves the lip line DOWN, - moves it UP
lip_y = int(np.clip(lip_y + lip_offset, 1, h - 2))

# 2. Restrict ROI to area above bucket lip (where dirt is)
margin = 6
roi = img[:max(lip_y + margin, 1), :].copy()
roi_gray = gray[:max(lip_y + margin, 1), :].copy()

# 3. Color-ish + darkness cue (HSV helps a bit, but keep it simple)
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
H, S, V = cv2.split(hsv)

# Dirt tends to be: moderate saturation, not super bright
mask_color = ((S > 20) & (V < 175)).astype(np.uint8) * 255

# 4. Texture cue (dirt is high texture; concrete is not)
lap = cv2.Laplacian(roi_gray, cv2.CV_64F)
texture = np.uint8(np.clip(np.abs(lap), 0, 255))
_, mask_tex = cv2.threshold(texture, 60, 255, cv2.THRESH_BINARY)

# Combine cues
mask = cv2.bitwise_and(mask_color, mask_tex)

# 5. Cleanup
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=6)

# Keep the main dirt blob
mask = largest_component(mask)

# 6. Put mask back into full image size
full_mask = np.zeros((h, w), dtype=np.uint8)
full_mask[:mask.shape[0], :] = mask

# 7. Create overlay (green highlight)
overlay = img.copy()
green = np.array([0, 255, 0], dtype=np.uint8)
overlay[full_mask == 255] = (0.4 * overlay[full_mask == 255] + 0.6 * green).astype(np.uint8)

# 8. Side-by-side proof image
side_by_side = cv2.hconcat([img, overlay])

cv2.imwrite("dirt_mask.png", full_mask)
cv2.imwrite("dirt_overlay.png", overlay)
cv2.imwrite("original_vs_highlighted.png", side_by_side)

cv2.imshow("Original vs Dirt Highlighted", side_by_side)
cv2.waitKey(0)
cv2.destroyAllWindows()