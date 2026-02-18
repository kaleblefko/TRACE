#!/usr/bin/env python3


import base64
import json
import os


import cv2
import numpy as np
import requests
import rclpy
from dotenv import load_dotenv
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from std_msgs.msg import String

class VLMNode(Node):
    def __init__(self):
        super().__init__("vlm_node")

        self.cam_node = GzNode()
        self.camera_topic = "/world/small_house/model/trace_drone_0/model/mono_cam/link/camera_link/sensor/camera/image"
        self.latest_image = None
        self.cam_node.subscribe(GzImage, self.camera_topic, self.gz_image_callback)

        self.pub = self.create_publisher(String, "/vlm/result", 10)


        # =======================
        # State
        # =======================
        self.vlm_processing = False
        self.vlm_check_interval = 20  # every 20 ticks (~1s at 20 Hz)
        self.vlm_check_counter = 0


        # =======================
        # Control loop at 20 Hz
        # =======================
        self.create_timer(0.05, self.control_loop)


        self.get_logger().info("VLMNode started.")


    # ------------------------------------------------------------------
    def gz_image_callback(self, msg: GzImage):
        """Receive RGB camera frames for VLM."""
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            img = img[:, :, ::-1]  # BGR -> RGB
            self.latest_image = cv2.resize(img, (320, 240))
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")


    # ------------------------------------------------------------------
    def control_loop(self):
        self.vlm_check_counter += 1
        if not self.vlm_processing:
            self.check_for_blue_ball_vlm()
        # if (self.vlm_check_counter % self.vlm_check_interval == 0
        #         and not self.vlm_processing):
        # self.check_for_blue_ball_vlm()


    # ------------------------------------------------------------------
    def check_for_blue_ball_vlm(self):
        """Use the VLM to check if a big blue sphere (yoga ball) is visible."""
        if self.latest_image is None:
            self.get_logger().debug("No RGB image available for VLM yet.")
            return


        self.vlm_processing = True
        try:
            success, buffer = cv2.imencode(".jpg", self.latest_image)
            if not success:
                self.get_logger().error("Failed to encode RGB image for VLM.")
                return


            img_b64 = base64.b64encode(buffer).decode("utf-8")


            uri = f"http://{os.getenv('OLLAMA_ENDPOINT')}/api/chat"
            system_prompt = """
            You are a visual inspector for a drone flying inside a house.


            Your task:
            Look at the image and decide if there is a clearly visible large blue sphere
            (like a blue yoga ball or exercise ball) anywhere in the frame.


            Output format:
            - Respond with exactly one word:
              - "yes"  -> if a large blue sphere / yoga ball is visible
              - "no"   -> if not


            Do not add any explanation, punctuation, or extra text.
            """


            data = {
                "model": os.getenv("OLLAMA_MODEL"),
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "images": [img_b64]},
                ],
            }


            self.get_logger().debug("Sending request to VLM for blue ball detection...")
            response = requests.post(uri, json=data, stream=True, timeout=10)


            full_reply = ""
            for line in response.iter_lines():
                if not line:
                    continue
                try:
                    json_data = json.loads(line)
                    full_reply += json_data.get("message", {}).get("content", "")
                    if json_data.get("done", False):
                        break
                except json.JSONDecodeError:
                    continue


            answer = full_reply.strip().lower()
            self.get_logger().info(f"VLM blue-ball reply: '{answer}'")


            if answer.startswith("yes"):
                result = "yes"
            elif answer.startswith("no"):
                result = "no"
            else:
                self.get_logger().warn(
                    f"Unexpected VLM reply: '{answer}'. Publishing 'unknown'."
                )
                result = "unknown"


            msg = String()
            msg.data = result
            self.pub.publish(msg)


        except Exception as e:
            self.get_logger().error(f"VLM blue-ball detection error: {e}")
        finally:
            self.vlm_processing = False




def main(args=None):
    rclpy.init(args=args)
    env_path = os.path.expanduser('~/TRACE/.env')
    load_dotenv(dotenv_path=env_path)


    node = VLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — shutting down VLMNode.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()




if __name__ == "__main__":
    main()