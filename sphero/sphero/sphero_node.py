#!/usr/bin/python3
import sys
import rclpy

try:
  from sphero import Sphero
except:
  from sphero.sphero import Sphero

def main(args=None):
  rclpy.init(args=args)

  rvr = Sphero()  # using node object
  logger = rvr.get_logger()

  if not rvr.initialize_rover():
    print('The Sphero RVR does not appear to be connected and powered on. Exiting.')
    return

  logger.info('Starting Sphero RVR')

  rvr.run()

  rvr.destroy_node()

  print("Sphero RVR stopped")


if __name__ == '__main__':
  try:
    main(sys.argv)
  except Exception as e:
    print(f'Exception: {e}')
