from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image  # Import the Image message type
import numpy as np
import cv2


def image_callback(msg: Image):
    # Depth data is R_FLOAT32 - single channel, 32-bit float
    depth_data = np.frombuffer(msg.data, dtype=np.float32)
    depth_img = depth_data.reshape((msg.height, msg.width))
    
    # Normalize depth for visualization (0-10 meters range)
    # Clip values beyond 10m and scale to 0-255
    depth_normalized = np.clip(depth_img, 0, 10.0) / 10.0 * 255
    depth_vis = depth_normalized.astype(np.uint8)
    
    # Apply colormap for better visualization
    depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    depth_small = cv2.resize(depth_colored, (480, 320))
    
    # Resize and show
    cv2.imshow("Depth Camera", depth_small)
    
    # Also show raw depth values at center pixel
    center_depth = depth_img[msg.height//2, msg.width//2]
    print(f"Center depth: {center_depth:.2f}m")
    
    cv2.waitKey(1)


node = Node()
topic = "/depth_camera"


print("Subscribing to:", topic)
node.subscribe(Image, topic, image_callback)


print("Rendering depth camera... Ctrl+C to exit.")
try:
    while True:
        pass
except KeyboardInterrupt:
    cv2.destroyAllWindows()
