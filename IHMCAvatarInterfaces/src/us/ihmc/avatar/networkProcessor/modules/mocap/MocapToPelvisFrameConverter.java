package us.ihmc.avatar.networkProcessor.modules.mocap;

import optiTrack.MocapRigidBody;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


// SMcCrory 2/1/2017 - this module is specific to localization tests performed on Atlas given a certain marker configuration

public class MocapToPelvisFrameConverter
{
   private boolean initialized = false;

   private ReferenceFrame mocapFrame = null;

   private static final double ballRadius = 0.006;
   private static final Vector3D markerPlateOriginInPelvisFrame = new Vector3D(0.17235, 0.0, 0.12888);
   private static final Vector3D plateOriginToMarker2InPelvisFrame = new Vector3D(0.005 + ballRadius, 0.045, 0.0);
   private static final RigidBodyTransform pelvisToMarker2Transform = new RigidBodyTransform();

   static
   {
      Vector3D marker2PositionInPelvisFrame = new Vector3D(markerPlateOriginInPelvisFrame);
      marker2PositionInPelvisFrame.add(plateOriginToMarker2InPelvisFrame);
      pelvisToMarker2Transform.setTranslation(marker2PositionInPelvisFrame);
      pelvisToMarker2Transform.invert();      
   }

   public MocapToPelvisFrameConverter()
   {

   }

   public boolean isInitialized()
   {
      return initialized;
   }

   public void initialize(ReferenceFrame pelvisFrame, MocapRigidBody markerRigidBody)
   {
      RigidBodyTransform marker2ToMocapTransform = new RigidBodyTransform(markerRigidBody.getOrientation(), markerRigidBody.getPosition());

      RigidBodyTransform worldToPelvisTransform = pelvisFrame.getTransformToWorldFrame();
      worldToPelvisTransform.invert();

      RigidBodyTransform worldToMocapTransform = new RigidBodyTransform();
      worldToMocapTransform.multiply(marker2ToMocapTransform);
      worldToMocapTransform.multiply(pelvisToMarker2Transform);
      worldToMocapTransform.multiply(worldToPelvisTransform);

      mocapFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("mocapFrame", ReferenceFrame.getWorldFrame(), worldToMocapTransform);
      initialized = true;
   }

   public void computePelvisToWorldTransform(MocapRigidBody mocapRigidBody, RigidBodyTransform pelvisToWorldTransformToPack)
   {
      RigidBodyTransform pelvisToMocapTransform = new RigidBodyTransform(mocapRigidBody.getOrientation(), mocapRigidBody.getPosition());
      pelvisToMocapTransform.multiply(pelvisToMarker2Transform);

      FramePose pelvisPose = new FramePose(mocapFrame, pelvisToMocapTransform);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisPose.getRigidBodyTransform(pelvisToWorldTransformToPack);
   }
   
   public ReferenceFrame getMocapFrame()
   {
      return mocapFrame;
   }
   
   public static RigidBodyTransform getPelvisToMarker2Transform()
   {
      return pelvisToMarker2Transform;
   }
}
