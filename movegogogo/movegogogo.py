'''
Date: 2022-03-10 16:28:38
LastEditors: houhuixiang
'''
import _thread
import time
import sys
import json
import glog as log
import logging

from numpy import empty
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from chassis_interfaces.srv import GetCurrentMapInfo
from chassis_interfaces.msg import NaviPointCmd, PointCoordinate, NaviParam, NaviWorkStatus

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

        self.navi_point = self.create_publisher(
            NaviPointCmd, '/navi_manager/navi_point', qos)

        self.navi_status = self.create_subscription(
            NaviWorkStatus, '/navi_manager/state_publisher/work_status',self.getNaviStatus, qos)

        self.current_map = self.create_client(
            GetCurrentMapInfo, '/map_manager/map/get_current_map')
        self.req = GetCurrentMapInfo.Request()
        self.navigation_state = "idle"
        self.is_reached = False

    def getNaviStatus(self,msg):
        self.navigation_state = msg.navigation_state
        if  self.navigation_state :
            log.info("navigation_state: %s" % self.navigation_state)
        if(self.navigation_state == "reached"):
            self.is_reached = True


    def getCurrentMap(self):
        self.future = self.current_map.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def pubNaviPoint(self, map_id, x, y, z, yaw):
        msg = NaviPointCmd()
        msg.map_id= map_id
        coordinate = PointCoordinate()
        coordinate.x = x
        coordinate.y = y
        coordinate.z = z
        coordinate.yaw = yaw
        msg.coordinate = coordinate
        
        navi_param = NaviParam()
        navi_param.is_forward = True
        navi_param.is_final_rotate=False
        navi_param.path_type=1
        navi_param.is_abs_reach=True
        msg.navi_param = navi_param
        
        self.navi_point.publish(msg)

def myThread(move,map_id, x, y, z, yaw, x2, y2, z2, yaw2):
    movegogogo = move
    run_count = 0
    while True:
        log.info("前往第一个点")
        movegogogo.pubNaviPoint(map_id, float(x), float(y), float(z), float(yaw))
        while movegogogo.is_reached != True:
            time.sleep(2)
        movegogogo.is_reached = False
        log.info("前往第二个点")
        movegogogo.pubNaviPoint(map_id, float(x2), float(y2), float(z2), float(yaw2))
        while movegogogo.is_reached != True:
            time.sleep(2)
        movegogogo.is_reached = False
        run_count = run_count+1
        log.info("来回次数总计： %d 次"% run_count)

def main(args=None):
    rclpy.init(args=args)
    initLogging("movegogogo.log", "info")

    movegogogo = MoveGoGoGo()
    # print("strat get current map")
    # response = movegogogo.get_current_map()
    # print("get current map id: %s" % response.map_id)

    map_id = input("请输入map_id: ")
    x = input("请输入x: ")
    y = input("请输入y: ")
    z = input("请输入z: ")
    yaw = input("请输入yaw: ")

    x2 = input("请输入第二个点x: ")
    y2 = input("请输入第二个点y: ")
    z2 = input("请输入第二个点z: ")
    yaw2 = input("请输入第二个点yaw: ")

    
    _thread.start_new_thread(myThread,(movegogogo,map_id,x,y,z,yaw,x2,y2,z2,yaw2))
    

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
