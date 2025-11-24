from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
import numpy as np
print(np.__version__)
import cv2

frame = 0

def image_callback(msg: Image):
    global frame
    frame += 1
    
    if frame % 10 != 0:
        return
    
    print(msg.data)
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape((msg.height, msg.width, 3))

    cv2.imshow("PX4 Downward Camera", img)
    cv2.waitKey(1)

node = Node()
topic = "/world/colored_blocks_world/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"

print("Subscribing to:", topic)
node.subscribe(msg_type=Image, topic=topic, callback=image_callback)

print("Rendering camera... Ctrl+C to exit.")
while True:
    pass