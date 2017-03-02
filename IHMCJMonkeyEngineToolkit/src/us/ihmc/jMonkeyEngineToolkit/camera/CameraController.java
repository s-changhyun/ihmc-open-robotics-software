package us.ihmc.jMonkeyEngineToolkit.camera;

import us.ihmc.euclid.transform.RigidBodyTransform;

public interface CameraController
{
   public void computeTransform(RigidBodyTransform cameraTransform);
   
   public double getHorizontalFieldOfViewInRadians();
   
   public double getClipNear();
   
   public double getClipFar();
   
   public void closeAndDispose();
}
