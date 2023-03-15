import depthai as dai

import cv2
import numpy as np

from datetime import timedelta, datetime
from pathlib import Path


class Recorder():

  def __init__(self, rgbRes: str, monoRes: str, fps: int) -> None:
    rgbResMap = {
        "12mp": dai.ColorCameraProperties.SensorResolution.THE_12_MP,
        "4k": dai.ColorCameraProperties.SensorResolution.THE_4_K,
        "1080p": dai.ColorCameraProperties.SensorResolution.THE_1080_P,
    }
    monoResMap = {
        "800p": dai.MonoCameraProperties.SensorResolution.THE_800_P,
        "720p": dai.MonoCameraProperties.SensorResolution.THE_720_P,
        "400p": dai.MonoCameraProperties.SensorResolution.THE_400_P,
    }

    self.rgbRes = rgbResMap[rgbRes]
    self.monoRes = monoResMap[monoRes]
    self.fps = fps

  def __initRecord(self, subpixel: bool, extended: bool):

    pipeline = dai.Pipeline()

    # Init nodes
    rgbCam = pipeline.create(dai.node.ColorCamera)
    leftCam = pipeline.create(dai.node.MonoCamera)
    rightCam = pipeline.create(dai.node.MonoCamera)

    stereo = pipeline.create(dai.node.StereoDepth)

    rgbOut = pipeline.create(dai.node.XLinkOut)
    rgbOut.setStreamName("rgbOut")
    depthOut = pipeline.create(dai.node.XLinkOut)
    depthOut.setStreamName("depthOut")

    # Link nodes
    rgbCam.video.link(rgbOut.input)
    leftCam.out.link(stereo.left)
    rightCam.out.link(stereo.right)
    stereo.depth.link(depthOut.input)

    # Config cameras
    rgbCam.setBoardSocket(dai.CameraBoardSocket.RGB)
    leftCam.setBoardSocket(dai.CameraBoardSocket.LEFT)
    rightCam.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    rgbCam.setResolution(self.rgbRes)
    leftCam.setResolution(self.monoRes)
    rightCam.setResolution(self.monoRes)

    rgbCam.setFps(self.fps)
    leftCam.setFps(self.fps)
    rightCam.setFps(self.fps)

    # Config stereo
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(subpixel)
    stereo.setExtendedDisparity(extended)

    self.__pipeline = pipeline
    self.__rgbCam = rgbCam

  def __initPreview(self, subpixel: bool, extended: bool):
    self.__depthWeight = 100
    self.__rgbWeight = 0

    pipeline = dai.Pipeline()

    # Init nodes
    rgbCam = pipeline.create(dai.node.ColorCamera)
    leftCam = pipeline.create(dai.node.MonoCamera)
    rightCam = pipeline.create(dai.node.MonoCamera)

    ## Preview
    stereo = pipeline.create(dai.node.StereoDepth)
    rgbOut = pipeline.create(dai.node.XLinkOut)
    disparityOut = pipeline.create(dai.node.XLinkOut)

    # Link nodes
    rgbCam.video.link(rgbOut.input)
    leftCam.out.link(stereo.left)
    rightCam.out.link(stereo.right)
    stereo.disparity.link(disparityOut.input)

    # Config cameras
    rgbCam.setBoardSocket(dai.CameraBoardSocket.RGB)
    leftCam.setBoardSocket(dai.CameraBoardSocket.LEFT)
    rightCam.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    rgbCam.setResolution(self.rgbRes)
    leftCam.setResolution(self.monoRes)
    rightCam.setResolution(self.monoRes)

    rgbCam.setFps(self.fps)
    leftCam.setFps(self.fps)
    rightCam.setFps(self.fps)

    # Config depth preview
    rgbOut.setStreamName("rgbOut")
    disparityOut.setStreamName("disparityOut")

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(subpixel)
    stereo.setExtendedDisparity(extended)
    maxDisparity = stereo.initialConfig.getMaxDisparity()

    self.__pipeline = pipeline
    self.__rgbCam = rgbCam
    self.__maxDisparity = maxDisparity

  def __getFrameTime(self, timestamp: timedelta) -> datetime:
    return datetime.now() - (dai.Clock.now() - timestamp)

  def __updateBlendWeights(self, depthPercent):
    self.__depthWeight = depthPercent
    self.__rgbWeight = 100 - self.__depthWeight

  def record(self, outputDirPath: Path, subpixel: bool, extended: bool):
    self.__initRecord(subpixel, extended)

    # Check output path
    if not outputDirPath.exists():
      outputDirPath.mkdir(parents=True, exist_ok=True)

    # Recording
    with dai.Device(self.__pipeline) as device:
      # RGB needs fixed focus to properly align with depth
      calibration = device.readCalibration()
      lensPosition = calibration.getLensPosition(dai.CameraBoardSocket.RGB)
      self.__rgbCam.initialControl.setManualFocus(lensPosition)

      # Output queues
      rgbOutQ = device.getOutputQueue(
          name="rgbOut", maxSize=self.fps, blocking=True)
      depthOutQ = device.getOutputQueue(
          name="depthOut", maxSize=self.fps, blocking=True)

      # Get first RGB frame accurate timestamp
      startTime = self.__getFrameTime(rgbOutQ.get().getTimestamp())
      timestamp = startTime.astimezone().isoformat().replace(":", ";")

      tag = f"[{timestamp}][{self.fps}FPS]"
      if subpixel:
        tag += "[Subpixel]"
      if extended:
        tag += "[Extended]"
      subDirPath = outputDirPath.joinpath(f"{tag}/")
      subDirPath.mkdir(exist_ok=True)

      # Backup calibration data
      calibrationPath = subDirPath.joinpath(f"calibration.json")
      calibration.eepromToJsonFile(calibrationPath)

      # Recording
      print(f"{startTime}, start recording (Press Ctrl+C to stop)")
      while True:
        try:
          recordingTime = datetime.now() - startTime
          print(f"\rRecording: {recordingTime}...", end="")

          rgbFrame = rgbOutQ.get()
          frameTime = self.__getFrameTime(rgbFrame.getTimestamp())
          timestamp = frameTime.astimezone().isoformat().replace(":", ";")
          rgbPath = subDirPath.joinpath(f"{timestamp}.png")
          cv2.imwrite(str(rgbPath), rgbFrame.getCvFrame())

          depthFrame = depthOutQ.get().getFrame()
          depthPath = rgbPath.with_suffix(".npy")
          np.save(str(depthPath), depthFrame)
        except KeyboardInterrupt:
          break
      print("\nStop recording...")

    print(f"Output files in: \"{outputDirPath.absolute()}\"")

  def preview(self, subpixel: bool, extended: bool):
    self.__initPreview(subpixel, extended)
    with dai.Device(self.__pipeline) as device:
      # RGB needs fixed focus to properly align with depth
      calibrationbData = device.readCalibration()
      lensPosition = calibrationbData.getLensPosition(dai.CameraBoardSocket.RGB)
      self.__rgbCam.initialControl.setManualFocus(lensPosition)

      rgbFrame = None
      disparityFrame = None

      # Trackbar adjusts blending ratio of rgb/depth
      windowName = "Preview"
      cv2.namedWindow(windowName)
      cv2.createTrackbar("Depth%", windowName, self.__depthWeight, 100,
                         self.__updateBlendWeights)

      print("Previewing... (Press Q on frame or Ctrl+C to stop)")
      while True:
        try:
          latestPacket = {}
          latestPacket["rgbOut"] = None
          latestPacket["disparityOut"] = None

          queueEvents = device.getQueueEvents(("rgbOut", "disparityOut"))
          for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
              latestPacket[queueName] = packets[-1]

          if latestPacket["rgbOut"] is not None:
            rgbFrame = latestPacket["rgbOut"].getCvFrame()

          if latestPacket["disparityOut"] is not None:
            disparityFrame = latestPacket["disparityOut"].getFrame()
            disparityFrame = (disparityFrame * 255. /
                              self.__maxDisparity).astype(np.uint8)
            disparityFrame = cv2.applyColorMap(disparityFrame,
                                               cv2.COLORMAP_BONE)
            disparityFrame = np.ascontiguousarray(disparityFrame)

          # Blend when both received
          if (rgbFrame is not None) and (disparityFrame is not None):
            # Need to have both frames in BGR format before blending
            if len(disparityFrame.shape) < 3:
              disparityFrame = cv2.cvtColor(disparityFrame, cv2.COLOR_GRAY2BGR)
            blended = cv2.addWeighted(rgbFrame,
                                      float(self.__rgbWeight) / 100,
                                      disparityFrame,
                                      float(self.__depthWeight) / 100, 0)
            cv2.imshow(windowName, blended)
            rgbFrame = None
            disparityFrame = None

          if cv2.waitKey(1) == ord("q"):
            break
        except KeyboardInterrupt:
          break

      print("Stop preview...")
      cv2.destroyAllWindows()
