package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PrismaticJoint extends OneDoFJoint
{
   private final FrameVector jointAxis;

   public PrismaticJoint(String name, RigidBody predecessor, ReferenceFrame beforeJointFrame, FrameVector jointAxis)
   {
      super(name, predecessor, beforeJointFrame, new PrismaticJointReferenceFrame(name, beforeJointFrame, jointAxis));
      this.jointAxis = new FrameVector(jointAxis);
      this.unitJointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame, jointAxis.getVector(), new Vector3D());
   }

   @Override
   public void setSuccessor(RigidBody successor)
   {
      this.successor = successor;

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      unitJointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame, jointAxis.getVector(), new Vector3D());

      unitSuccessorTwist = new Twist(unitJointTwist);
      unitSuccessorTwist.changeBaseFrameNoRelativeTwist(predecessorFrame);
      unitSuccessorTwist.changeBodyFrameNoRelativeTwist(successorFrame);
      unitSuccessorTwist.changeFrame(successorFrame);

      unitPredecessorTwist = new Twist(unitSuccessorTwist);
      unitPredecessorTwist.invert();
      unitPredecessorTwist.changeFrame(predecessorFrame);

      unitJointAcceleration = new SpatialAccelerationVector(afterJointFrame, beforeJointFrame, afterJointFrame, jointAxis.getVector(), new Vector3D());

      unitSuccessorAcceleration = new SpatialAccelerationVector(unitJointAcceleration);
      unitSuccessorAcceleration.changeBaseFrameNoRelativeAcceleration(predecessorFrame);
      unitSuccessorAcceleration.changeBodyFrameNoRelativeAcceleration(successorFrame);
      unitSuccessorAcceleration.changeFrameNoRelativeMotion(successorFrame);

      unitPredecessorAcceleration = new SpatialAccelerationVector(unitSuccessorAcceleration);
      unitPredecessorAcceleration.invert();
      unitPredecessorAcceleration.changeFrameNoRelativeMotion(predecessorFrame); // actually, there is relative motion, but not in the directions that matter

      setMotionSubspace(unitSuccessorTwist);
   }

   @Override
   public FrameVector getJointAxis()
   {
      return new FrameVector(jointAxis);
   }

   @Override
   public void getJointAxis(FrameVector axisToPack)
   {
      axisToPack.setIncludingFrame(jointAxis);
   }

   @Override
   public boolean isPassiveJoint()
   {
      return false;
   }
}
