import argparse


def parseCli():
  description = "Record RGB and aligned depth with DepthAI OAK-D"
  parser = argparse.ArgumentParser(description=description)
  parser.add_argument(
      "-p",
      "--preview",
      action="store_const",
      const=True,
      default=False,
      help="Show preview, nothing will be recorded")
  parser.add_argument(
      "-o",
      "--out",
      type=str,
      metavar="",
      default="output",
      help="Output directory path, default value is \"output\"")
  parser.add_argument(
      "-m",
      "--mode",
      type=str,
      choices=["density", "accuracy"],
      metavar="",
      default="density",
      help="Depth mode: [\"density\", \"accuracy\"], default value is \"density\""
  )
  parser.add_argument(
      "-s",
      "--subpixel",
      action="store_const",
      const=True,
      default=False,
      help="Enable subpixel disparity")
  parser.add_argument(
      "-e",
      "--extended",
      action="store_const",
      const=True,
      default=False,
      help="Enable extended disparity")
  parser.add_argument(
      "-f",
      "--fps",
      type=int,
      metavar="",
      default=30,
      help="Specify FPS, default value is \"30\"")
  parser.add_argument(
      "--rgbres",
      type=str,
      choices=["12mp", "4k", "1080p"],
      metavar="",
      default="1080p",
      help="RGB camera resolution: [\"12mp\", \"4k\", \"1080p\"], default value is \"1080p\""
  )
  parser.add_argument(
      "--monores",
      type=str,
      choices=["800p", "720p", "400p"],
      metavar="",
      default="800p",
      help="Mono camera resolution: [\"800p\", \"720p\", \"400p\"], default value is \"720p\""
  )
  args = parser.parse_args()
  return args
