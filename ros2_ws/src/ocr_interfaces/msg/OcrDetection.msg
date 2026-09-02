# A single OCR detection result (one text region)
# corners: 4 corner points of the quadrilateral bounding box
#   [x1, y1, x2, y2, x3, y3, x4, y4] in pixel coordinates
float32[8] corners

# Recognized text content
string text

# Confidence score (0.0 - 1.0)
float32 confidence
