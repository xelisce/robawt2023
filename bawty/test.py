import cv2
import numpy as np

# Callback function for mouse events
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Prompt user to enter a value
        text = input("Enter a value: ")

        # Draw the text on the image
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow(window_name, image)

# Create a window and set the callback function for mouse events
window_name = 'Drawing on Image'
cv2.namedWindow(window_name)
cv2.setMouseCallback(window_name, mouse_callback)

# Load an image
image = 255 * np.ones((300, 500, 3), dtype=np.uint8)

while True:
    # Display the image with any text added
    cv2.imshow(window_name, image)

    # Wait for a key press
    key = cv2.waitKey(1) & 0xFF

    # Check if the 'q' key was pressed to quit
    if key == ord('q'):
        break

# Destroy the window
cv2.destroyAllWindows()