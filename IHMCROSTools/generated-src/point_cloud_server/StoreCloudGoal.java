package point_cloud_server;

public interface StoreCloudGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "point_cloud_server/StoreCloudGoal";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n# The storage name of the point cloud.\nstring name\n\n# The topic on which to capture a point cloud message. \n# If this is empty, then \'cloud\' will be used instead.\nstring topic\n\n# A point cloud to store.\nsensor_msgs/PointCloud2 cloud\n\n# If not empty, transforms the cloud to this frame before storing.\nstring storage_frame_id\n\n# If not empty, transforms the cloud to this frame in the return result.\nstring result_frame_id\n\n# A flag to determine whether to reply with the cloud.\nint32 action\n\n# Will get a message on topic, or store cloud.\nint32 STORE=0\n\n# Will get a message on a topic if it is provided, save, and return it;\n# otherwise just returns the existing cloud.\nint32 GET=1\n\n# Topic and cloud are ignored, just removes cloud from the server.\nint32 CLEAR=2\n\n";
  static final int STORE = 0;
  static final int GET = 1;
  static final int CLEAR = 2;
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getTopic();
  void setTopic(java.lang.String value);
  sensor_msgs.PointCloud2 getCloud();
  void setCloud(sensor_msgs.PointCloud2 value);
  java.lang.String getStorageFrameId();
  void setStorageFrameId(java.lang.String value);
  java.lang.String getResultFrameId();
  void setResultFrameId(java.lang.String value);
  int getAction();
  void setAction(int value);
}
