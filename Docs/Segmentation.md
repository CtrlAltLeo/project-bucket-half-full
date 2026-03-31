# Segmentation.py Line-by-Line Explanation

This document explains what each part of **Segmentation.py** is doing and connects it to the output images produced during the segmentation process. The goal of the script is to isolate the dirt inside the loader bucket so that it can be used for later calculations, such as estimating volume.

Source file: **Segmentation.py**

---

## 1. Import libraries

```python
import cv2
import numpy as np
```

These two lines import the tools used throughout the script.

- `cv2` is OpenCV, which handles the image processing steps.
- `numpy` is used for array math, since images are stored as grids of pixel values.

This does not create an output image yet. It just brings in the libraries needed for the rest of the code.

---

## 2. Define a helper function to keep the largest object

```python
def largest_component(mask: np.ndarray) -> np.ndarray:
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return mask
    # Skip label 0 (background). Pick largest by area.
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    out = np.zeros_like(mask)
    out[labels == largest] = 255
    return out
```

This function looks at a binary mask and finds all connected white regions. It then keeps only the biggest one.

### What each line is doing

- `cv2.connectedComponentsWithStats(...)` labels every separate white region in the mask.
- `if num_labels <= 1:` checks whether there is anything besides background.
- `largest = 1 + np.argmax(...)` finds the biggest non-background region.
- `out = np.zeros_like(mask)` creates a blank mask.
- `out[labels == largest] = 255` fills in only the largest connected region.
- `return out` sends that cleaned mask back to the main program.

### Why this matters

After thresholding and cleanup, there may still be small white spots that are not actually dirt. This function removes those smaller pieces and keeps the main dirt region.

### Connected image

- **`16_mask_largest.png`** shows the result after this function is applied.

---

## 3. Read the input image

```python
img = cv2.imread(r"C:\Users\paulp\OneDrive\Documents\UMary\ENR488\roi.png")
if img is None:
    raise FileNotFoundError("Could not read image")
```

This loads the original image into memory.

- `cv2.imread(...)` reads the image file.
- The `if img is None:` check makes sure the image was actually found.
- If the image cannot be loaded, the program stops and gives an error.

### Connected image

- The original input can be seen in **`19_side_by_side.png`** on the left side.

---

## 4. Get image size and convert to grayscale

```python
h, w = img.shape[:2]
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
```

These lines prepare the image for edge and texture processing.

- `h, w = img.shape[:2]` stores the image height and width.
- `cv2.cvtColor(..., cv2.COLOR_BGR2GRAY)` converts the image from color to grayscale.

### Why this matters

Grayscale simplifies the image into brightness values only. That makes it easier to detect edges and textures without worrying about color.

### Connected image

- The grayscale version used for later calculations is shown in **`07_roi_gray.png`** after the region of interest is cropped.

---

## 5. Detect the bucket lip using horizontal edges

```python
# 1. Detect bucket top edge as strongest horizontal edge row
sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
row_energy = np.abs(sobel_y).sum(axis=1)
```

This section tries to find the top edge of the bucket.

### What each line is doing

- `cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)` calculates vertical brightness changes across the image. Since the derivative is taken in the y-direction, strong horizontal edges become easy to detect.
- `np.abs(sobel_y).sum(axis=1)` adds up the edge strength across each row. This gives one value per row, showing how strong the horizontal edge is there.

### Why this matters

The top of the bucket creates a strong horizontal line in the image. Finding that line helps separate the dirt area from the loader arms and hardware below it.

---

## 6. Search for the strongest horizontal edge in a reasonable area

```python
# Search in a reasonable band (avoid image border + machine arms lower down)
y0 = int(h * 0.15)
y1 = int(h * 0.70)
lip_y = y0 + np.argmax(row_energy[y0:y1])
```

This section limits the search to the middle part of the image so the code does not accidentally pick a random strong edge near the top border or lower machine parts.

### What each line is doing

- `y0 = int(h * 0.15)` sets the upper search limit.
- `y1 = int(h * 0.70)` sets the lower search limit.
- `np.argmax(row_energy[y0:y1])` finds the row with the strongest edge inside that band.
- `lip_y = y0 + ...` converts that local row index back into the full image coordinate.

### Why this matters

This makes the lip detection more stable by focusing only on rows where the bucket edge is likely to be.

---

## 7. Adjust the detected lip position

```python
lip_offset = 100   # + moves the lip line DOWN, - moves it UP
lip_y = int(np.clip(lip_y + lip_offset, 1, h - 2))
```

This section manually shifts the detected lip line.

- `lip_offset = 100` pushes the line downward.
- `np.clip(...)` keeps the line inside valid image bounds.

### Why this matters

The strongest edge is not always exactly where you want the cutoff for the dirt region. This offset lets you tune the crop so it better matches the visible bucket opening.

### Connected image

- **`05_lip_detected.png`** shows the detected bucket lip as a red line.

---

## 8. Restrict the image to the region above the bucket lip

```python
# 2. Restrict ROI to area above bucket lip (where dirt is)
margin = 6
roi = img[:max(lip_y + margin, 1), :].copy()
roi_gray = gray[:max(lip_y + margin, 1), :].copy()
```

This section crops the image so that only the upper area, where the dirt is located, is kept for further processing.

### What each line is doing

- `margin = 6` adds a small buffer below the cutoff line.
- `roi = ...` crops the color image.
- `roi_gray = ...` crops the grayscale image.

### Why this matters

This removes unnecessary lower parts of the image, such as the machine arms and mechanisms, which could interfere with segmentation.

### Connected images

- **`06_roi_color.png`** shows the cropped color region.
- **`07_roi_gray.png`** shows the cropped grayscale region.

---

## 9. Convert the cropped region to HSV color space

```python
# 3. Color-ish + darkness cue (HSV helps a bit, but keep it simple)
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
H, S, V = cv2.split(hsv)
```

This converts the cropped image into HSV format.

### What each part means

- `H` is hue, which represents the basic color type.
- `S` is saturation, which shows how strong or rich the color is.
- `V` is value, which represents brightness.

### Why this matters

HSV can make it easier to separate materials by color and brightness than standard RGB or BGR. In this project, dirt tends to be darker and somewhat saturated compared to bright background areas.

### Connected images

- **`08_hsv_H.png`** shows hue.
- **`09_hsv_S.png`** shows saturation.
- **`10_hsv_V.png`** shows brightness.

---

## 10. Build a color-based dirt mask

```python
# Dirt tends to be: moderate saturation, not super bright
mask_color = ((S > 20) & (V < 175)).astype(np.uint8) * 255
```

This creates a binary mask using simple color and brightness rules.

### What this line is doing

- `S > 20` keeps pixels with at least some color richness.
- `V < 175` keeps pixels that are not too bright.
- `&` means both conditions must be true.
- `.astype(np.uint8) * 255` converts the result into a black-and-white image where white means the pixel passed the test.

### Why this matters

This is the first attempt at isolating dirt based on how it looks. It is a rough filter that tries to remove bright concrete or shiny machine parts.

### Connected image

- **`11_mask_color.png`** shows the color-based mask.

---

## 11. Measure texture using the Laplacian operator

```python
# 4. Texture cue (dirt is high texture; concrete is not)
lap = cv2.Laplacian(roi_gray, cv2.CV_64F)
texture = np.uint8(np.clip(np.abs(lap), 0, 255))
_, mask_tex = cv2.threshold(texture, 60, 255, cv2.THRESH_BINARY)
```

This section uses texture instead of just color.

### What each line is doing

- `cv2.Laplacian(...)` detects rapid intensity changes in the grayscale image.
- `np.abs(lap)` takes the absolute value so both bright-to-dark and dark-to-bright changes count.
- `np.clip(..., 0, 255)` keeps values in display range.
- `np.uint8(...)` converts the result into standard image format.
- `cv2.threshold(texture, 60, 255, cv2.THRESH_BINARY)` turns the texture image into a binary mask. Pixels above the threshold become white.

### Why this matters

Dirt usually has a rough, uneven texture, while smoother surfaces like concrete or metal have less texture. This helps identify the dirt even when color alone is not enough.

### Connected images

- **`12_texture_abs_laplacian.png`** shows the texture strength.
- **`13_mask_tex.png`** shows the thresholded texture mask.

---

## 12. Combine the color mask and texture mask

```python
# Combine cues
mask = cv2.bitwise_and(mask_color, mask_tex)
```

This keeps only pixels that pass both the color rule and the texture rule.

### Why this matters

Using both cues together reduces false positives. A region now has to look like dirt in terms of brightness/color and also have dirt-like texture.

### Connected image

- **`14_mask_combined.png`** shows the combined mask.

---

## 13. Clean the mask with morphology

```python
# 5. Cleanup
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=6)
```

This section improves the mask shape.

### What each line is doing

- `cv2.getStructuringElement(...)` creates an elliptical kernel used for cleanup.
- `MORPH_CLOSE` fills small holes and connects nearby white regions.
- `MORPH_OPEN` removes small white noise and smooths the result.

### Why this matters

The raw combined mask is usually noisy. Morphological operations help turn it into a cleaner, more solid region that better represents the dirt pile.

### Connected image

- **`15_mask_morph.png`** shows the cleaned mask after morphology.

---

## 14. Keep only the main dirt blob

```python
# Keep the main dirt blob
mask = largest_component(mask)
```

This sends the cleaned mask into the helper function from earlier.

### Why this matters

If small leftover regions remain after cleanup, this step removes them and keeps the main connected dirt region.

### Connected image

- **`16_mask_largest.png`** shows the result after keeping only the largest component.

---

## 15. Put the mask back into the full original image size

```python
# 6. Put mask back into full image size
full_mask = np.zeros((h, w), dtype=np.uint8)
full_mask[:mask.shape[0], :] = mask
```

Earlier, the image was cropped to the region of interest. This step places the final cropped mask back into an image the same size as the original.

### What each line is doing

- `np.zeros((h, w), dtype=np.uint8)` creates a blank full-size mask.
- `full_mask[:mask.shape[0], :] = mask` copies the segmented ROI mask into the top part of that full-size image.

### Why this matters

This allows the final result to match the original image dimensions, which makes it easier to visualize or use later with the original stereo image.

### Connected image

- **`17_full_mask.png`** shows the mask restored to full image size.

---

## 16. Create a green overlay on the original image

```python
# 7. Create overlay (green highlight)
overlay = img.copy()
green = np.array([0, 255, 0], dtype=np.uint8)
overlay[full_mask == 255] = (0.4 * overlay[full_mask == 255] + 0.6 * green).astype(np.uint8)
```

This section highlights the detected dirt region on the original image.

### What each line is doing

- `overlay = img.copy()` makes a copy of the original image.
- `green = np.array([0, 255, 0], dtype=np.uint8)` defines a bright green color.
- `overlay[full_mask == 255] = ...` blends green into the pixels where the mask is white.

### Why this matters

This gives a visual proof of segmentation by showing exactly which pixels were identified as dirt.

### Connected image

- **`18_overlay.png`** shows the green highlighted region.

---

## 17. Create a side-by-side comparison image

```python
# 8. Side-by-side proof image
side_by_side = cv2.hconcat([img, overlay])
```

This joins the original image and the highlighted image into one comparison output.

### Why this matters

It makes it easier to directly compare the input and the segmentation result in a single image.

### Connected image

- **`19_side_by_side.png`** shows the original image on the left and the segmented result on the right.

---

## 18. Save the final outputs

```python
cv2.imwrite("dirt_mask.png", full_mask)
cv2.imwrite("dirt_overlay.png", overlay)
cv2.imwrite("original_vs_highlighted.png", side_by_side)
```

These lines save the final result images to disk.

### Output files

- `dirt_mask.png` stores the final binary mask.
- `dirt_overlay.png` stores the green overlay.
- `original_vs_highlighted.png` stores the side-by-side comparison.

These are useful for documenting results and showing proof that the segmentation process worked.

---

## 19. Display the result on screen

```python
cv2.imshow("Original vs Dirt Highlighted", side_by_side)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

This opens a window showing the side-by-side result.

### What each line is doing

- `cv2.imshow(...)` displays the image.
- `cv2.waitKey(0)` waits until a key is pressed.
- `cv2.destroyAllWindows()` closes the display window.

### Why this matters

This allows the user to immediately inspect the segmentation output without needing to open the saved files manually.

---

## Final summary of the full pipeline

In simple terms, this script follows a clear segmentation pipeline:

1. Read the image.
2. Convert it to grayscale.
3. Detect the bucket lip to find where the useful region ends.
4. Crop the image to the area where the dirt is expected.
5. Use HSV values to find pixels that look like dirt.
6. Use texture analysis to find pixels that behave like dirt.
7. Combine those two clues.
8. Clean the mask with morphology.
9. Keep only the main dirt region.
10. Place the result back onto the full image.
11. Create a green overlay for visualization.
12. Save and display the final results.

This makes the code a good proof-of-concept for segmentation because each step narrows the image down until the dirt region is isolated in a way that is easy to understand and visualize.
