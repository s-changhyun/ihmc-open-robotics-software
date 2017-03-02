package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ConstantGoalDetectorBehaviorService extends GoalDetectorBehaviorService
{
   private final HumanoidReferenceFrames referenceFrames;
   private final Point3D constantGoalLocationInMidFeetZUpFrame = new Point3D();

   public ConstantGoalDetectorBehaviorService(HumanoidReferenceFrames referenceFrames, Point3D constantGoalLocationInMidFeetZUpFrame,
                                              CommunicationBridgeInterface communicationBridge)
   {
      super(ConstantGoalDetectorBehaviorService.class.getSimpleName(), communicationBridge);
      this.referenceFrames = referenceFrames;
      this.constantGoalLocationInMidFeetZUpFrame.set(constantGoalLocationInMidFeetZUpFrame);
   }

   @Override
   public boolean getGoalHasBeenLocated()
   {
      return true;
   }

   @Override
   public void getReportedGoalPoseWorldFrame(FramePose framePoseToPack)
   {
      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FramePoint goalPosition = new FramePoint(midFeetZUpFrame, constantGoalLocationInMidFeetZUpFrame);
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());

      FrameOrientation goalOrientation = new FrameOrientation(midFeetZUpFrame);
      goalOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      framePoseToPack.setPosition(goalPosition);
      framePoseToPack.setOrientation(goalOrientation);
   }

   @Override
   public void doThreadAction()
   {
   }

   @Override
   public void initialize()
   {
   }

}
