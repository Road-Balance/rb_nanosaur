import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from spider_msgs.msg import RCControl


import json
import asyncio

from .RTCPub.rtc_pub import RTCDataChannel
from .RTCCam.rtc_cam import WebCam, CSICam
from .RTCCam.dual_cam import DualCSICam


class SpiderCar(RTCDataChannel):

    def __init__(self, channel_id, cam_id=0):
        super().__init__(channel=channel_id)

        # self._cam = WebCam(width=640, height=480, camID=cam_id)
        # self._cam = CSICam(width=640, height=480, camID=0)
        self._cam = DualCSICam(
            cam_num1=0, cam_num2=1, width=640, height=480, flip1=2, flip2=2
        )
        self._peer.video.putSubscription(self._cam)  # 카메라 설정

    async def receiver(self):
        while True:
            self._motion = await self._dc_subscriber.get()
            print(self._motion)

            if "motion" not in self._motion:
                continue
            if "key" not in self._motion["motion"]:
                continue
            if "value" not in self._motion["motion"]:
                continue

            key = self._motion["motion"]["key"]
            val = self._motion["motion"]["value"]

            if key == "forward":
                self._cam.set_mode(True)
                self._is_back = False
            elif key == "back":
                self._cam.set_mode(False)

    async def sender(self):
        ir_metric = {"ir": {"key": "detectTime", "value": 0.0}}
        prev_black = False

        while True:
            is_black, cur_timestamp = self._myIR.get_lineinfo()
            # print(is_black, cur_timestamp)
            if prev_black and not is_black:
                ir_metric["ir"]["value"] = cur_timestamp
                self._dc_metric.put_nowait(ir_metric)
            prev_black = is_black
            await asyncio.sleep(1)

    def __del__(self):
        print("Deletion")
        if self._cam:
            self._cam.close()
        return super().__del__()


class JoyPublisher(Node):
    def __init__(self):
        super().__init__("rtc_server_node")

        self._data = RCControl()
        self._spider_car = SpiderCar(channel_id="c40hipepjh65aeq6ndj0", cam_id=0)
        self._loop = asyncio.get_event_loop()

        self._subscriber = self.create_subscription(
            Header, 'ir_val', self.sub_callback, 10
        )
        self._publisher = self.create_publisher(RCControl, "spider_control", 10)

        self.run()

    def sub_callback(self, msg):
        time_stamp = msg.stamp
        self.get_logger().info(time_stamp)

    async def receiver(self):
        while True:
            self._motion = await self._spider_car._dc_subscriber.get()
            print(self._motion)

            if "motion" not in self._motion:
                continue
            if "key" not in self._motion["motion"]:
                continue
            if "value" not in self._motion["motion"]:
                continue

            key = self._motion["motion"]["key"]
            val = self._motion["motion"]["value"]

            if key == "forward":
                self._spider_car._cam.set_mode(True)
                self._is_back = False
            elif key == "back":
                self._spider_car._cam.set_mode(False)

            self._data = RCControl()

            self._data.throttle = 255
            self._data.steering = 255

            self._publisher.publish(self._data)

    async def sender(self):
        ir_metric = {"ir": {"key": "detectTime", "value": 0.0}}
        prev_black = False

        while True:
            self._spider_car._dc_metric.put_nowait(ir_metric)
            await asyncio.sleep(0)

    def run(self):
        try:
            asyncio.ensure_future(self._spider_car.connect())
            asyncio.ensure_future(self.receiver())
            # asyncio.ensure_future(self.sender())
            self._loop.run_forever()
        except Exception as e:
            print(e)
        finally:
            print("Done...")


def main(args=None):
    rclpy.init(args=args)
    rw = JoyPublisher()

    rclpy.spin(rw)

    rw.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
