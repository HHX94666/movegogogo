'''
Date: 2022-03-10 16:28:38
LastEditors: houhuixiang
'''
import _thread
import time
import sys
import glog as log
import logging
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist


def initLogging(logFilename, e):

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s-%(levelname)s-%(message)s',
        datefmt='%y-%m-%d %H:%M',
        filename=logFilename,
        filemode='a')

    filehandler = logging.FileHandler(logFilename, encoding='utf-8')
    logging.getLogger().addHandler(filehandler)
    log = logging.exception(e)
    return log


class MoveGoGoGo(Node):
    charge_state = 0

    def __init__(self):
        super().__init__('movegogogo')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        log.setLevel("INFO")

        self.cmd_vel = self.create_publisher(
            Twist, '/cmd_vel', qos)

    def pubCmdVel(self, x):
        msg = Twist()
        msg.linear.x = x
        self.cmd_vel.publish(msg)
        # log.info("pubCmdVel linear: %f" % x)


def main(args=None):
    rclpy.init(args=args)
    initLogging("pile_test.log", "info")

    movegogogo = MoveGoGoGo()

    in_x = input("请输入移动速度: ")
    in_t = input("请输入移动时间: ")
    x = float(in_x)
    t = int(in_t)
    print("int_x: %f  int_t: %d" % (x, t))

    i = t = t*5
    count=0
    while True:

        if(i > 0):
            movegogogo.pubCmdVel(x)
            time.sleep(0.2)
            i = i-1
        elif(i > -t):
            movegogogo.pubCmdVel(-x)
            time.sleep(0.2)
            i = i-1
        else:
            i = t
            count = count+1
            log.info("往返计数器： %d 次" % count)
    try:
        rclpy.spin(movegogogo)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        movegogogo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
