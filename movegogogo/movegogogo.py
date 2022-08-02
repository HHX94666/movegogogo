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
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# from geometry_msgs.msg import Twis
from chassis_interfaces.srv import GetCurrentMapInfo
from chassis_interfaces.msg import NaviPointCmd, PointCoordinate, NaviParam

navi_point = '{"traceId": "123456789","method": "thing.service.downChannel","params": {"method": "deviceDataUpChannel","data": {"op": "publish","topic": "/navi_manager/navi_point","msg": {"map_id": "","point_id": "","point_type": "","coordinate": {"x": 0.0,"y": 0.0,"yaw": 0.0},"navi_param": {"is_forward": true,"is_final_rotate": false,"path_type": 1,"is_abs_reach": true, }}},"appCode": "FAVV9MVY"}, "version": "1.6.0"}'
test = '{"traceId": "123456789"}'
navi_point_json = json.loads(test)
print(navi_point_json)
# navi_point_json = json.loads(navi_point)


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

        self.current_map = self.create_client(
            GetCurrentMapInfo, '/map_manager/map/get_current_map')
        self.req = GetCurrentMapInfo.Request()

    def get_current_map(self):
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

def main(args=None):
    rclpy.init(args=args)
    initLogging("movegogogo.log", "info")

    movegogogo = MoveGoGoGo()
    
    movegogogo.pubNaviPoint("map_id", 11.2, 12.1, 0.1, -0.2)
    print("strat get current map")
    response = movegogogo.get_current_map()
    print("get current map id: %s" % response.map_id)

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
