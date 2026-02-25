import cv2
import numpy as np

def largest_component(mask: np.ndarray) -> np.ndarray:
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return mask
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    out = np.zeros_like(mask)
    out[labels == largest] = 255
    return out

def nothing(_):
    pass

img_path = r"C:\Users\paulp\OneDrive\Documents\UMary\ENR488\Segmentation\little_left.png"  # change if needed
img = cv2.imread(img_path)
if img is None:
    raise FileNotFoundError(f"Could not read {img_path}")

h, w = img.shape[:2]
gray_full = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# --- Auto-estimate bucket lip row (strong horizontal edge) ---
sobel_y = cv2.Sobel(gray_full, cv2.CV_64F, 0, 1, ksize=3)
row_energy = np.abs(sobel_y).sum(axis=1)
y0 = int(h * 0.15)
y1 = int(h * 0.70)
lip_y_auto = y0 + int(np.argmax(row_energy[y0:y1]))

cv2.namedWindow("Tuner", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Tuner", 1200, 650)

# Trackbars (start values tuned to be reasonable defaults)
cv2.createTrackbar("Lip offset (+/- px)", "Tuner", 50, 100, nothing)   # 50 means 0 offset
cv2.createTrackbar("S min", "Tuner", 20, 255, nothing)
cv2.createTrackbar("V max", "Tuner", 175, 255, nothing)
cv2.createTrackbar("Texture thr", "Tuner", 18, 80, nothing)
cv2.createTrackbar("Kernel", "Tuner", 9, 31, nothing)                 # odd sizes look nicer
cv2.createTrackbar("Close iters", "Tuner", 2, 6, nothing)
cv2.createTrackbar("Open iters", "Tuner", 1, 6, nothing)
cv2.createTrackbar("Keep largest blob", "Tuner", 1, 1, nothing)       # 1 = yes, 0 = no

print("Controls:")
print("- Drag sliders to tune segmentation live.")
print("- Press 's' to save: mask, overlay, side-by-side.")
print("- Press 'q' or ESC to quit.\n")

while True:
    lip_offset = cv2.getTrackbarPos("Lip offset (+/- px)", "Tuner") - 50
    s_min      = cv2.getTrackbarPos("S min", "Tuner")
    v_max      = cv2.getTrackbarPos("V max", "Tuner")
    tex_thr    = cv2.getTrackbarPos("Texture thr", "Tuner")
    k          = cv2.getTrackbarPos("Kernel", "Tuner")
    close_it   = cv2.getTrackbarPos("Close iters", "Tuner")
    open_it    = cv2.getTrackbarPos("Open iters", "Tuner")
    keep_big   = cv2.getTrackbarPos("Keep largest blob", "Tuner")

    # force odd kernel >= 3
    k = max(3, k)
    if k % 2 == 0:
        k += 1

    lip_y = int(np.clip(lip_y_auto + lip_offset, 1, h-1))
    roi = img[:lip_y, :]
    roi_gray = gray_full[:lip_y, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(hsv)

    # Color/darkness cue
    mask_color = ((S > s_min) & (V < v_max)).astype(np.uint8) * 255

    # Texture cue
    lap = cv2.Laplacian(roi_gray, cv2.CV_64F)
    texture = np.uint8(np.clip(np.abs(lap), 0, 255))
    _, mask_tex = cv2.threshold(texture, tex_thr, 255, cv2.THRESH_BINARY)

    mask = cv2.bitwise_and(mask_color, mask_tex)

    # Morph cleanup
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    if close_it > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=close_it)
    if open_it > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=open_it)

    if keep_big == 1:
        mask = largest_component(mask)

    # Put ROI mask back into full size
    full_mask = np.zeros((h, w), dtype=np.uint8)
    full_mask[:lip_y, :] = mask

    # Overlay
    overlay = img.copy()
    overlay[full_mask == 255] = (0.4 * overlay[full_mask == 255] + 0.6 * np.array([0, 255, 0])).astype(np.uint8)

    # Draw lip line for debugging (red)
    overlay_line = overlay.copy()
    cv2.line(overlay_line, (0, lip_y), (w-1, lip_y), (0, 0, 255), 2)

    side_by_side = cv2.hconcat([img, overlay_line])

    # Display mask small-ish as well (stacked view)
    mask_bgr = cv2.cvtColor(full_mask, cv2.COLOR_GRAY2BGR)
    mask_bgr = cv2.resize(mask_bgr, (side_by_side.shape[1], side_by_side.shape[0] // 3))
    view = cv2.vconcat([side_by_side, mask_bgr])

    cv2.imshow("Tuner", view)

    key = cv2.waitKey(1) & 0xFF
    if key in (27, ord('q')):
        break
    if key == ord('s'):
        cv2.imwrite("tuned_dirt_mask.png", full_mask)
        cv2.imwrite("tuned_dirt_overlay.png", overlay_line)
        cv2.imwrite("tuned_original_vs_highlighted.png", side_by_side)
        print("Saved: tuned_dirt_mask.png, tuned_dirt_overlay.png, tuned_original_vs_highlighted.png")

cv2.destroyAllWindows()