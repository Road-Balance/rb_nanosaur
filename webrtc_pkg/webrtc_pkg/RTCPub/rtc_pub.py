import sys
import ssl
import json
import asyncio
import aiortc
import aiohttp

sys.path.append("..")

from aiortc.rtp import RtpPacket
from rtcbot import RTCConnection, CVCamera
from aiortc import RTCConfiguration, RTCIceServer, RTCIceGatherer


class RTCPublisher(object):
    def __init__(self, channel="c0dko1upjh69daabh9pg"):
        self._servers = [
            # order
            # 1. stun
            # 2. turn - udp
            # 3, turn - tcp
            RTCIceServer(urls="stun:cobot.center:3478"),
            RTCIceServer(
                urls="turn:cobot.center:3478?transport=udp",
                username="teamgrit",
                credential="teamgrit8266",
            ),
            RTCIceServer(
                urls="turn:cobot.center:3478?transport=tcp",
                username="teamgrit",
                credential="teamgrit8266",
            ),
        ]  # 사용할 ice서버 정보
        # self._cam = WebCam(width=640, height=480, camID=0)
        self._peer = RTCConnection(
            rtcConfiguration=RTCConfiguration(iceServers=self._servers)
        )

        # self._peer.video.putSubscription(self._cam)  # 카메라 설정
        # 일반적인 SSLContext 객체를 생성합니다.
        self._sslcontext = ssl.create_default_context()
        self._sslcontext.check_hostname = False  # 호스트 이름확인을 활성화하지 않는다. (이름검사x)
        self._sslcontext.verify_mode = ssl.CERT_NONE  # 모든인증서 허용

        try:
            self._loop = asyncio.get_event_loop()
        except RuntimeError as ex:
            if "There is no current event loop in thread" in str(ex):
                self._loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self._loop)

        # self._loop = asyncio.get_event_loop()
        self._channel = channel

    def __del__(self):
        self._peer.close()
        # self._cam.close()

    async def validateWS(self, ws):
        async for msg in ws:  # websocket으로 받은 메시지 msg
            if msg.type == aiohttp.WSMsgType.TEXT:  # 정상적인 메시지일때
                msg_type = msg.json()["type"]  # 메시지의 타입을 저장한다.
                print("type is " + msg_type)
                if msg.json()["type"] == "answer":  # offer에 대한 대답 메시지일 때,
                    message = msg.json()["data"]
                    print(message)
                    json_answer = {
                        "type": "answer",
                        "sdp": message,
                    }  # 받은 메시지를 기반으로 remotedescription을 만든다.
                    # remoteDescription을 등록한다.
                    await self._peer.setRemoteDescription(json_answer)
                    print("success")
                if msg.json()["type"] == "joins":  # 통신이 끊어지거나 새로 연결된경우
                    pass
                if msg.data == "close":  # 웹소켓 종료 메시지일 대
                    await ws.close()  # 종료후 break한다.
                    break
            # 에러메시지가 도착할 경우, 임시 주소로 통신하지 않는다.
            elif msg.type == aiohttp.WSMsgType.ERROR:
                print(msg)
                print(msg.data)
                await ws.close()
                break

    """
    복수개의 ice-ufrag, ice-pwd 값을 그것들의 가장 첫번째 값으로 치환하여 같은 값을 가지도록 한다. 2021.5.25
    """

    def sdpUpdate(self, pure_sdp):
        lines = pure_sdp.split("\r\n")  # 해당 sdp를 수정하기 위해서 줄별로 나눈다.
        ufrags = []
        pwds = []
        # ice-ufrag들과 ice-pwd값을 모두 추출하자.
        for line in lines[:-1]:
            if -1 != line.find("a=ice-ufrag"):
                point = line.split(":")
                ufrags.append(point[1])
            if -1 != line.find("a=ice-pwd"):
                point = line.split(":")
                pwds.append(point[1])

        # result1_sdp = pure_sdp.replace(ufrags[-1], ufrags[0])
        # result2_sdp = result1_sdp.replace(pwds[-1], pwds[0])
        result_sdp = pure_sdp
        for ufrag in ufrags:
            result_sdp = result_sdp.replace(ufrag, ufrags[0])
        for pwd in pwds:
            result_sdp = result_sdp.replace(pwd, pwds[0])
        return result_sdp

    async def connect(self):
        # 로컬 SDP를 얻는다.
        localDescription = await self._peer.getLocalDescription()
        pure_sdp = localDescription["sdp"]  # sdp의 주 내용들을 추출한다.
        result2_sdp = self.sdpUpdate(pure_sdp)
        print("-----offer begin")
        print(result2_sdp)
        print("-----offer end")
        # websocket통신 초기화
        async with aiohttp.ClientSession() as session:
            # 미리 정해진 채널에서 websocket을 초기화한다.
            async with session.ws_connect(  # channel id should be replaced / TODO : bitrate check
                url=f"wss://cobot.center:8267/live/ws/pub?channel={self._channel}&brate=800&ktime=100",
                ssl_context=self._sslcontext,
            ) as ws:
                # offer단계에서 전송할 sdp정보를 가진 json메시지를 만든다.
                json_offer = {"type": "offer", "data": result2_sdp}
                await ws.send_json(json_offer)  # json메시지를 전송한다.
                await self.validateWS(ws)
                return True

    async def sender(self):
        pass

    async def receiver(self):
        pass
        while True:
            frame = await self.m_subscriber.get()
            self.m_display.put_nowait(frame)

    def run(self):
        try:
            asyncio.ensure_future(self.connect())
            self._loop.run_forever()
        except Exception as e:
            print(e)
        finally:
            print("Done...")


class RTCSubscriber(RTCPublisher):
    def __init__(self, channel="c0dko1upjh69daabh9pg"):
        super().__init__(channel=channel)

    async def connect(self):
        # 로컬 SDP를 얻는다.
        localDescription = await self._peer.getLocalDescription()
        pure_sdp = localDescription["sdp"]  # sdp의 주 내용들을 추출한다.
        # result2_sdp = self.sdpUpdate(pure_sdp)
        result2_sdp = pure_sdp
        print(result2_sdp)

        # websocket통신 초기화
        async with aiohttp.ClientSession() as session:
            # 미리 정해진 채널에서 websocket을 초기화한다.
            async with session.ws_connect(  # channel id should be replaced / TODO : bitrate check
                url="wss://cobot.center:8267/live/ws/sub?channel=c0dko1upjh69daabh9pg",
                ssl_context=self._sslcontext,
            ) as ws:
                # offer단계에서 전송할 sdp정보를 가진 json메시지를 만든다.
                json_offer = {"type": "offer", "data": result2_sdp}
                await ws.send_json(json_offer)  # json메시지를 전송한다.
                await self.validateWS(ws)


class RTCDataChannel(RTCPublisher):
    def __init__(self, channel="c0dko1upjh69daabh9pg", streamID="stream"):
        super().__init__(channel=channel)

        # peer객체에 "stream"이라는 이름의 데이터 체널을 추가한다.
        self._dc_metric = self._peer.addDataChannel("metric")
        self._dc_stream = self._peer.addDataChannel("stream")
        self._dc_subscriber = self._dc_stream.subscribe()

    def __del__(self):
        self._dc_stream.close()
        return super().__del__()

    async def sender(self):
        while True:
            await asyncio.sleep(1)
            self._dc_metric.put_nowait("hi metric")

    async def receiver(self):
        while True:
            data = await self._dc_subscriber.get()
            print(data)

    def run(self):
        try:
            asyncio.ensure_future(self.connect())
            asyncio.ensure_future(self.receiver())
            self._loop.run_forever()
        except Exception as e:
            print(e)
        finally:
            print("Done...")

# For Profile
# myRTCBot = RTCWebCamPublisher(channel="c0kscq6pjh69daabictg")
# myRTCBot.run()

if __name__ == "__main__":
    # myRTCBot = RTCWebCamPublisher(channel="c0dko1upjh69daabh9pg")
    myRTCBot = RTCPublisher(channel="c0kscq6pjh69daabictg")
    # myRTCBot = RTCDataChannel(channel="c0kscq6pjh69daabictg")
    myRTCBot.run()

