#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
from geometry_msgs.msg import PolygonStamped, Point, Point32
from visualization_msgs.msg import Marker
import time

import tf2_ros
import numpy as np
import ros_numpy
import pcl  # Use this one for tx2: https://github.com/PonyPC/python-pcl-jetson
from alphashape import alphashape
from shapely.geometry import Polygon, LineString
from shapely.ops import cascaded_union
from pyquaternion import Quaternion

cloud_pub = rospy.Publisher('/brink/out/cloud', PointCloud2, queue_size=10)
poly_pub = rospy.Publisher('/brink/out/poly', PolygonStamped, queue_size=10)
hull_pub = rospy.Publisher('/brink/out/hull', PolygonStamped, queue_size=10)
lines_pub = rospy.Publisher('/brink/out/lines', Marker, queue_size=10)
range_pub = rospy.Publisher('/brink/out/range', Float64, queue_size=10)
range_text_pub = rospy.Publisher('/brink/out/range_text', Marker, queue_size=10)

class tfSource:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def lookup(self, _from, _to):
        try:
            tf = self.tfBuffer.lookup_transform(_from, _to, rospy.Time()).transform
            quat = Quaternion(tf.rotation.w,tf.rotation.x,tf.rotation.y,tf.rotation.z)
            trans = np.array([tf.translation.x,tf.translation.y,tf.translation.z])
            return quat, trans
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None


class Brinkmanship:
    def __init__(self, odom_frame_id, filter_size=[0.05, 0.05, 0.05]):
        self.tf_source = tfSource()
        self.odom_frame_id = odom_frame_id
        self.filter_size = filter_size

        self.cloud_sub = rospy.Subscriber('/brink/in/cloud', PointCloud2, self.cloud_handler)

    def msg2np(self, msg):
        pc = ros_numpy.numpify(msg)
        h  = pc.shape[0]
        w  = pc.shape[1]
        pts = np.zeros((h*w, 3), dtype=np.float32)
        pts[:,0] = np.array(pc['x']).flatten()
        pts[:,1] = np.array(pc['y']).flatten()
        pts[:,2] = np.array(pc['z']).flatten()
        return pts

    def np2msg(self, pts):
        h = pts.shape[0]
        w = pts.shape[1]
        data = np.zeros(pts.shape[0], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        data['x'] = pts[:,0]
        data['y'] = pts[:,1]
        data['z'] = pts[:,2]
        msg = ros_numpy.msgify(PointCloud2, data)
        return msg

    def orient_cloud(self, pts, odom_frame_id, camera_frame_id):
        tf = self.tf_source.lookup(odom_frame_id, camera_frame_id)

        R = tf[0].rotation_matrix
        T = tf[1]

        if tf is None:
            print("Brinkmanship failed to find transform from camera to odom.")
            # Better handle this...
            return None

        h = pts.shape[0]
        w = pts.shape[1]
        pts = pts.reshape((w*h, 3))
        pts = np.dot(R, pts.transpose()).transpose()
        pts += T
        pts = pts.reshape((h,w,3))
        return pts

    def ray_plane_intersection(self, ray_orig, ray_dir, coeffs):
        normal = coeffs[0:3]
        t = -(ray_orig.dot(normal) + coeffs[3]) / ray_dir.dot(normal)
        return ray_dir*t + ray_orig

    def project_from_origin_to_plane(self, pts, coeffs):
        ray_orig = np.array([0.0, 0.0, 0.0])
        new_pts = np.empty(pts.shape)
        for i,pt in enumerate(pts):
            ray_dir = (pt-ray_orig)/np.linalg.norm(pt-ray_orig)
            new_pts[i,:] = self.ray_plane_intersection(ray_orig, ray_dir, coeffs)
        return new_pts

    def project_point_to_plane(self, pt, plane):
        n = plane[0:3]
        d = plane[3]
        return pt - (n*pt + d) * n

    def cloud_handler(self, msg):
        # Save for posterity.
        camera_frame_id = msg.header.frame_id

        # Convert ros PointCloud2 msg to numpy array.
        pts = self.msg2np(msg)

        # Range filter
        ranges = np.linalg.norm(pts, axis=1)
        mask = np.logical_and(0.02 < ranges, ranges < 1.0)
        pts = pts[mask,:]

        # Convert to pcl PointCloud
        pcl_pts = pcl.PointCloud()
        pcl_pts.from_array(pts)

        # Apply VoxelGrid filter
        f = pcl_pts.make_voxel_grid_filter()
        f.set_leaf_size(self.filter_size[0], self.filter_size[1], self.filter_size[2])
        pcl_pts = f.filter()

        # RANSAC fit a plane
        seg = pcl_pts.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_normal_distance_weight(0.1)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(100)
        seg.set_distance_threshold(0.15)

        try:
          indices, model = seg.segment()
          model = np.array(model)
        except:
          print("Brinkmanship: Failed to segment model.")
          return

        if( model[2] > 0.0 ):
            model *= -1

        # Publish the plane polygon
        (A, B, C, D) = (model[0], model[1], model[2], model[3])
        ctr = np.mean(pcl_pts, axis=0)

        poly_msg = PolygonStamped()
        poly_msg.header = msg.header
        poly_msg.header.frame_id = camera_frame_id
        poly_msg.polygon.points.append(Point(-1, -1, (-A-B+D)/-C))
        poly_msg.polygon.points.append(Point(-1, +1, (-A+B+D)/-C))
        poly_msg.polygon.points.append(Point(+1, +1, (+A+B+D)/-C))
        poly_msg.polygon.points.append(Point(+1, -1, (+A-B+D)/-C))
        poly_pub.publish(poly_msg)

        # Smash all points into the plane.
        pts = self.project_from_origin_to_plane(np.asarray(pcl_pts), model)

        # Publish the plane cloud
        cloud_msg = self.np2msg(pts)
        cloud_msg.header = msg.header
        cloud_msg.header.frame_id = camera_frame_id
        cloud_pub.publish(cloud_msg)

        # Move planar points to 2d.
        orig = self.project_point_to_plane(np.array([0,0,0]), model)
        e0 = self.project_point_to_plane(np.array([1,0,0]), model) - orig
        e2 = model[0:3]
        e1 = np.cross(e2, e0)
        basis = np.zeros((3,3))
        basis[:,0] = e0 / np.linalg.norm(e0)
        basis[:,1] = e1 / np.linalg.norm(e1)
        basis[:,2] = e2 / np.linalg.norm(e2)
        pts -= orig
        pts = (basis.transpose().dot(pts.transpose())).transpose() # These transposes are B.S., but it works...
        pts_2d = pts[:,0:2]

        # Compute the inverse for later
        inv_basis = np.linalg.inv(basis)

        # Compute 2d alpha shape
        concave_hull, edge_points = alphashape(pts_2d, alpha=0.1)

        # Draw lines through the alpha shape to see how far you can drive.
        lines = []
        for angle in range(-20, 30+1, 10):
            ro = np.array([0,0.0])
            rd = 10*np.array([np.sin(angle*np.pi/180.0),np.cos(angle*np.pi/180.0)])
            line = LineString([(ro[0]-rd[0], ro[1]-rd[1]), (ro[0]+rd[0], ro[1]+rd[1])])
            int_line = concave_hull.intersection(line)
            try:
                lines.append(int_line.coords)
            except:
                pass

        lines_msg = Marker()
        lines_msg.header = msg.header
        lines_msg.header.frame_id = camera_frame_id
        lines_msg.type = Marker.LINE_LIST
        lines_msg.pose.orientation.w = 1.0
        lines_msg.scale.x = .01; # Line width
        lines_msg.color.r = 1.0; # Use red lines
        lines_msg.color.a = 1.0; # Use opaque lines
        for l in lines:
            xyz = np.array([l[0][0],l[0][1],0]).transpose()
            xyz = xyz.dot(inv_basis)
            xyz += orig
            p0 = Point(xyz[0], xyz[1], xyz[2])
            lines_msg.points.append(p0)

            xyz = np.array([l[1][0],l[1][1],0]).transpose()
            xyz = xyz.dot(inv_basis)
            xyz += orig
            p1 = Point(xyz[0], xyz[1], xyz[2])
            lines_msg.points.append(p1)
        lines_pub.publish(lines_msg)

        try:
          # Publish the estimated range to a brink.
          dists = [np.linalg.norm(np.array([l[0][0]-l[1][0],l[0][1]-l[1][1]])) for l in lines]
          brink_range = np.min(dists)
          range_pub.publish(brink_range)
          
          # Publish a string version of the estimated range to a brink (for rviz).
          range_text_msg = Marker()
          range_text_msg.header.frame_id = "base_link"
          range_text_msg.type = 9

          # Normally white.
          range_text_msg.color.r = 1.0;
          range_text_msg.color.g = 1.0;
          range_text_msg.color.b = 1.0;
          range_text_msg.color.a = 1.0;
      
          # Yellow if getting worried. Red if way too close!
          if brink_range < 0.2:
            range_text_msg.color.g = 0.0;
            range_text_msg.color.b = 0.0;
          elif brink_range < 0.5:
            range_text_msg.color.b = 0.0

          range_text_msg.scale.z = 0.25;
          range_text_msg.pose.position.z = 1.5;
          range_text_msg.text = "BRINK: {:03f} m".format(brink_range)
          range_text_pub.publish(range_text_msg)

          # Put 2d alpha shape back in the camera_frame and publish it as a polygon.
          hull_msg = PolygonStamped()
          hull_msg.header = msg.header
          hull_msg.header.frame_id = camera_frame_id

          xs = concave_hull.exterior.coords.xy[0]
          ys = concave_hull.exterior.coords.xy[1]
          for x,y in zip(xs,ys):
              xyz = np.array([x,y,0]).transpose()
              xyz = xyz.dot(inv_basis)
              xyz += orig
              hull_msg.polygon.points.append(Point32(xyz[0], xyz[1], xyz[2]))
          hull_pub.publish(hull_msg)
        except:
            pass



if __name__=="__main__":
    rospy.init_node('brink')

    brink = Brinkmanship(odom_frame_id='odom', filter_size=[0.1,0.1,0.1])

    rospy.spin()
