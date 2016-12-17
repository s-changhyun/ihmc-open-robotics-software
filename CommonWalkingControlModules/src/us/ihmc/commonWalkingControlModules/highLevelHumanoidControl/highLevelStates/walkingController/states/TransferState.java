package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class TransferState extends WalkingState
{
   protected final RobotSide transferToSide;

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final HighLevelHumanoidControllerToolbox momentumBasedController;
   protected final WalkingFailureDetectionControlModule failureDetectionControlModule;

   protected final CenterOfMassHeightManager comHeightManager;
   protected final BalanceManager balanceManager;
   protected final PelvisOrientationManager pelvisOrientationManager;
   protected final FeetManager feetManager;

   private final FramePoint2d desiredICPLocal = new FramePoint2d();
   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final FramePoint2d desiredCMP = new FramePoint2d();

   private final FramePoint2d filteredDesiredCoP = new FramePoint2d();
   private final FramePoint nextExitCMP = new FramePoint();

   public TransferState(RobotSide transferToSide, WalkingStateEnum transferStateEnum, WalkingMessageHandler walkingMessageHandler,
         HighLevelHumanoidControllerToolbox momentumBasedController, HighLevelControlManagerFactory managerFactory,
         WalkingFailureDetectionControlModule failureDetectionControlModule, YoVariableRegistry parentRegistry)
   {
      super(transferStateEnum, parentRegistry);
      this.transferToSide = transferToSide;
      this.walkingMessageHandler = walkingMessageHandler;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.momentumBasedController = momentumBasedController;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   @Override
   public void doAction()
   {
      feetManager.updateContactStatesInDoubleSupport(transferToSide);

      switchToToeOffIfPossible();

      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      comHeightManager.setSupportLeg(transferToSide);
   }

   @Override
   public boolean isDone()
   {
      if (!balanceManager.isICPPlanDone())
         return false;
      balanceManager.getCapturePoint(capturePoint2d);
      FrameConvexPolygon2d supportPolygonInWorld = momentumBasedController.getBipedSupportPolygons().getSupportPolygonInWorld();
      boolean isICPInsideSupportPolygon = supportPolygonInWorld.isPointInside(capturePoint2d);

      if (!isICPInsideSupportPolygon)
         return true;
      else
         return balanceManager.isTransitionToSingleSupportSafe(transferToSide);
   }

   public boolean isStopWalkingSafe()
   {
      return balanceManager.isTransitionToStandingSafe();
   }

   public void switchToToeOffIfPossible()
   {
      RobotSide trailingLeg = transferToSide.getOppositeSide();
      // the only case left for determining the contact state of the trailing foot
      if (feetManager.getCurrentConstraintType(trailingLeg) != ConstraintType.TOES)
      {
         balanceManager.getDesiredCMP(desiredCMP);
         balanceManager.getDesiredICP(desiredICPLocal);
         balanceManager.getCapturePoint(capturePoint2d);

         boolean doToeOff = feetManager.checkIfToeOffSafe(trailingLeg, desiredCMP, desiredICPLocal, capturePoint2d);

         if (doToeOff)
         {
            momentumBasedController.getFilteredDesiredCenterOfPressure(momentumBasedController.getContactableFeet().get(trailingLeg), filteredDesiredCoP);
            balanceManager.getNextExitCMP(nextExitCMP);

            feetManager.computeToeOffContactPoint(trailingLeg, nextExitCMP, filteredDesiredCoP);
            feetManager.requestToeOff(trailingLeg);
            momentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states
         }
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      balanceManager.clearICPPlan();

      feetManager.initializeContactStatesForDoubleSupport(transferToSide);
      momentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      failureDetectionControlModule.setNextFootstep(walkingMessageHandler.peek(0));

      balanceManager.resetPushRecovery();

      double transferTime;

      boolean isPreviousStateDoubleSupport = isInitialTransfer();
      if (isPreviousStateDoubleSupport)
         transferTime = balanceManager.getInitialTransferDuration();
      else
         transferTime = walkingMessageHandler.getTransferTime();

      pelvisOrientationManager.setTrajectoryTime(transferTime);
   }

   public boolean isInitialTransfer()
   {
      return getPreviousState().getStateEnum() == WalkingStateEnum.STANDING;
   }

   @Override
   public void doTransitionOutOfAction()
   {
      feetManager.reset();
   }
}