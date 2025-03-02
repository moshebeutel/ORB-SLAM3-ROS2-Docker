
import struct
from sensor_msgs.msg import PointCloud2, PointField
def parse_point_clou(msg):
        """ Extracts (x, y, z) points from PointCloud2 message """
        points = []
        for i in range(msg.width):
            offset = i * msg.point_step
            x = struct.unpack_from("f", msg.data, offset)[0]
            y = struct.unpack_from("f", msg.data, offset + 4)[0]
            z = struct.unpack_from("f", msg.data, offset + 8)[0]
            points.append((x, y, z))
        return points



msg: PointCloud2 = PointCloud2()
msg.header.stamp.sec = 8808
msg.header.stamp.nanosec = 280000000
msg.header.frame_id = "map"
msg.height = 1
msg.width = 20693
msg.fields = [
    PointField(name='x', offset=0, datatype=7, count=1),
    PointField(name='y', offset=4, datatype=7, count=1),
    PointField(name='z', offset=8, datatype=7, count=1)
]
msg.is_bigendian = False
msg.point_step = 16
msg.row_step = 331088
msg.is_dense = True
msg.data = [ 0, 0, 140, 192, 0, 0, 72, 192, 0, 0, 0, 190, 0, 0, 128, 63, 0, 0, 132
, 192, 0, 0, 72, 192, 0, 0, 0, 190, 0, 0, 128, 63, 0, 0, 164, 192, 0, 0, 24, 192, 0
, 0, 192, 190, 0, 0, 128, 63, 0, 0, 172, 192, 0, 0, 8, 192, 0, 0, 192, 190, 0, 0, 128, 63
, 0, 0, 164, 192, 0, 0, 8, 192, 0, 0, 192, 190, 0, 0, 128, 63, 0, 0, 164, 192, 0, 0, 24, 192
, 0, 0, 0, 190, 0, 0, 128, 63, 0, 0, 172, 192, 0, 0, 8, 192, 0, 0, 0, 190, 0, 0, 128, 63, 0
, 0, 164, 192, 0, 0, 8, 192, 0, 0, 0, 190, 0, 0, 128, 63,0,0,0,0, 0,0,0,0,0,0,0,0]

points = parse_point_clou(msg)