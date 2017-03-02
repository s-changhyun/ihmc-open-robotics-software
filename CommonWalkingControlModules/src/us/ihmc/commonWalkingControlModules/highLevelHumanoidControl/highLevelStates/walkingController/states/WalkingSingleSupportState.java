package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class WalkingSingleSupportState extends SingleSupportState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private Footstep nextFootstep;
   private final FootstepTiming footstepTiming = new FootstepTiming();
   private double swingTime;

   private final FramePose actualFootPoseInWorld = new FramePose(worldFrame);
   private final FramePose desiredFootPoseInWorld = new FramePose(worldFrame);
   private final FramePoint nextExitCMP = new FramePoint();

   private final HighLevelHumanoidControllerToolbox momentumBasedController;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;

   private final DoubleYoVariable remainingSwingTimeAccordingToPlan = new DoubleYoVariable("remainingSwingTimeAccordingToPlan", registry);
   private final DoubleYoVariable estimatedRemainingSwingTimeUnderDisturbance = new DoubleYoVariable("estimatedRemainingSwingTimeUnderDisturbance", registry);
   private final DoubleYoVariable icpErrorThresholdToSpeedUpSwing = new DoubleYoVariable("icpErrorThresholdToSpeedUpSwing", registry);

   private final BooleanYoVariable finishSingleSupportWhenICPPlannerIsDone = new BooleanYoVariable("finishSingleSupportWhenICPPlannerIsDone", registry);
   private final BooleanYoVariable minimizeAngularMomentumRateZDuringSwing = new BooleanYoVariable("minimizeAngularMomentumRateZDuringSwing", registry);

   public WalkingSingleSupportState(RobotSide supportSide, WalkingMessageHandler walkingMessageHandler, HighLevelHumanoidControllerToolbox momentumBasedController,
         HighLevelControlManagerFactory managerFactory, WalkingControllerParameters walkingControllerParameters,
         WalkingFailureDetectionControlModule failureDetectionControlModule, YoVariableRegistry parentRegistry)
   {
      super(supportSide, WalkingStateEnum.getWalkingSingleSupportState(supportSide), walkingMessageHandler, momentumBasedController, managerFactory,
            parentRegistry);

      this.momentumBasedController = momentumBasedController;
      this.failureDetectionControlModule = failureDetectionControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();

      icpErrorThresholdToSpeedUpSwing.set(walkingControllerParameters.getICPErrorThresholdToSpeedUpSwing());
      finishSingleSupportWhenICPPlannerIsDone.set(walkingControllerParameters.finishSingleSupportWhenICPPlannerIsDone());
      minimizeAngularMomentumRateZDuringSwing.set(walkingControllerParameters.minimizeAngularMomentumRateZDuringSwing());
   }


   @Override
   public void doAction()
   {
      super.doAction();

      boolean icpErrorIsTooLarge = balanceManager.getICPErrorMagnitude() > icpErrorThresholdToSpeedUpSwing.getDoubleValue();
      boolean requestSwingSpeedUp = icpErrorIsTooLarge;

      if (walkingMessageHandler.hasRequestedFootstepAdjustment())
      {
         boolean footstepHasBeenAdjusted = walkingMessageHandler.pollRequestedFootstepAdjustment(nextFootstep);
         if (footstepHasBeenAdjusted)
         {
            walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
            failureDetectionControlModule.setNextFootstep(nextFootstep);
            updateFootstepParameters();

            feetManager.replanSwingTrajectory(swingSide, nextFootstep, swingTime, true);

            balanceManager.updateICPPlanForSingleSupportDisturbances();
         }

      }
      else if (balanceManager.useICPOptimization()) // TODO figure out a way of combining the two following modules
      {
         boolean footstepIsBeingAdjusted = balanceManager.checkAndUpdateFootstepFromICPOptimization(nextFootstep);

         if (footstepIsBeingAdjusted)
         {
            requestSwingSpeedUp = true;
            walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
            failureDetectionControlModule.setNextFootstep(nextFootstep);
            updateFootstepParameters();

            feetManager.replanSwingTrajectory(swingSide, nextFootstep, swingTime, true);

            balanceManager.updateICPPlanForSingleSupportDisturbances();
         }
      }
      else if (balanceManager.isPushRecoveryEnabled())
      {
         boolean footstepHasBeenAdjusted = balanceManager.checkAndUpdateFootstep(nextFootstep);
         if (footstepHasBeenAdjusted)
         {
            requestSwingSpeedUp = true;
            walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
            failureDetectionControlModule.setNextFootstep(nextFootstep);
            updateFootstepParameters();

            feetManager.replanSwingTrajectory(swingSide, nextFootstep, swingTime, false);

            walkingMessageHandler.reportWalkingAbortRequested();
            walkingMessageHandler.clearFootsteps();
            footstepTiming.setTimings(swingTime, walkingMessageHandler.getDefaultTransferTime());

            balanceManager.clearICPPlan();
            balanceManager.setICPPlanSupportSide(supportSide);
            balanceManager.addFootstepToPlan(nextFootstep, footstepTiming);
            balanceManager.updateICPPlanForSingleSupportDisturbances();
         }
      }

      if (requestSwingSpeedUp)
      {
         double swingTimeRemaining = requestSwingSpeedUpIfNeeded();
         balanceManager.updateSwingTimeRemaining(swingTimeRemaining);
      }

      walkingMessageHandler.clearFootTrajectory();

      switchToToeOffIfPossible();
   }

   @Override
   public boolean isDone()
   {
      if (super.isDone())
         return true;

      return finishSingleSupportWhenICPPlannerIsDone.getBooleanValue() && balanceManager.isICPPlanDone();
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      double defaultSwingTime = walkingMessageHandler.getDefaultSwingTime();
      double defaultTransferTime = walkingMessageHandler.getDefaultTransferTime();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();

      if (balanceManager.isRecoveringFromDoubleSupportFall())
      {
         swingTime = defaultSwingTime;
         footstepTiming.setTimings(swingTime, defaultTransferTime);
         nextFootstep = balanceManager.createFootstepForRecoveringFromDisturbance(swingSide, defaultSwingTime);
         nextFootstep.setTrajectoryType(TrajectoryType.DEFAULT);
         walkingMessageHandler.reportWalkingAbortRequested();
         walkingMessageHandler.clearFootsteps();
      }
      else
      {
         swingTime = walkingMessageHandler.getNextSwingTime();
         footstepTiming.set(walkingMessageHandler.peekTiming(0));
         nextFootstep = walkingMessageHandler.poll();
      }

      updateFootstepParameters();

      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringSwing.getBooleanValue());

      balanceManager.setNextFootstep(nextFootstep);

      balanceManager.addFootstepToPlan(nextFootstep, footstepTiming);
      balanceManager.addFootstepToPlan(walkingMessageHandler.peek(0), walkingMessageHandler.peekTiming(0));
      balanceManager.addFootstepToPlan(walkingMessageHandler.peek(1), walkingMessageHandler.peekTiming(1));
      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.initializeICPPlanForSingleSupport(defaultSwingTime, defaultTransferTime, finalTransferTime);

      if (balanceManager.isRecoveringFromDoubleSupportFall())
      {
         balanceManager.updateICPPlanForSingleSupportDisturbances();
         balanceManager.requestICPPlannerToHoldCurrentCoMInNextDoubleSupport();
      }

      feetManager.requestSwing(swingSide, nextFootstep, swingTime);

      nextFootstep.getPose(desiredFootPoseInWorld);
      desiredFootPoseInWorld.changeFrame(worldFrame);

      actualFootPoseInWorld.setToZero(fullRobotModel.getEndEffectorFrame(swingSide, LimbName.LEG));
      actualFootPoseInWorld.changeFrame(worldFrame);
      walkingMessageHandler.reportFootstepStarted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();

      balanceManager.minimizeAngularMomentumRateZ(false);

      actualFootPoseInWorld.setToZero(fullRobotModel.getEndEffectorFrame(swingSide, LimbName.LEG)); // changed Here Nicolas
      actualFootPoseInWorld.changeFrame(worldFrame);
      walkingMessageHandler.reportFootstepCompleted(swingSide, actualFootPoseInWorld);
      walkingMessageHandler.registerCompletedDesiredFootstep(nextFootstep);
   }

   private final FramePoint2d desiredCMP = new FramePoint2d(worldFrame);
   private final FramePoint2d filteredDesiredCoP = new FramePoint2d(worldFrame);
   private final FramePoint2d currentICP = new FramePoint2d(worldFrame);
   private final FramePoint2d desiredICP = new FramePoint2d(worldFrame);

   public void switchToToeOffIfPossible()
   {
      if (feetManager.doToeOffIfPossibleInSingleSupport() && feetManager.isInFlatSupportState(supportSide))
      {
         balanceManager.getDesiredCMP(desiredCMP);
         balanceManager.getCapturePoint(currentICP);
         balanceManager.getDesiredICP(desiredICP);

         if (feetManager.checkIfToeOffSafeSingleSupport(nextFootstep, desiredCMP, currentICP, desiredICP, balanceManager.isOnExitCMP()))
         {
            momentumBasedController.getFilteredDesiredCenterOfPressure(momentumBasedController.getContactableFeet().get(supportSide), filteredDesiredCoP);
            balanceManager.getNextExitCMP(nextExitCMP);

            feetManager.computeToeOffContactPoint(supportSide, nextExitCMP, filteredDesiredCoP);
            feetManager.requestToeOff(supportSide);
         }
      }
   }

   /**
    * Request the swing trajectory to speed up using {@link ICPPlanner#estimateTimeRemainingForStateUnderDisturbance(double, FramePoint2d)}.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @return the current swing time remaining for the swing foot trajectory
    */
   private double requestSwingSpeedUpIfNeeded()
   {
      remainingSwingTimeAccordingToPlan.set(balanceManager.getTimeRemainingInCurrentState());
      estimatedRemainingSwingTimeUnderDisturbance.set(balanceManager.estimateTimeRemainingForSwingUnderDisturbance());

      if (estimatedRemainingSwingTimeUnderDisturbance.getDoubleValue() > 1.0e-3)
      {
         double swingSpeedUpFactor = remainingSwingTimeAccordingToPlan.getDoubleValue() / estimatedRemainingSwingTimeUnderDisturbance.getDoubleValue();
         return feetManager.requestSwingSpeedUp(swingSide, swingSpeedUpFactor);
      }
      else if (remainingSwingTimeAccordingToPlan.getDoubleValue() > 1.0e-3)
      {
         return feetManager.requestSwingSpeedUp(swingSide, Double.POSITIVE_INFINITY);
      }
      return remainingSwingTimeAccordingToPlan.getDoubleValue();
   }

   private void updateFootstepParameters()
   {
      pelvisOrientationManager.setTrajectoryTime(swingTime);
      pelvisOrientationManager.setWithUpcomingFootstep(nextFootstep);

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForSingleSupport(nextFootstep, swingSide);
      transferToAndNextFootstepsData.setTransferFromDesiredFootstep(walkingMessageHandler.getLastDesiredFootstep(supportSide));
      double extraToeOffHeight = 0.0;
      if (feetManager.willDoToeOff(nextFootstep, swingSide))
         extraToeOffHeight = feetManager.getWalkOnTheEdgesManager().getExtraCoMMaxHeightWithToes();
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);

      // Update the contact states based on the footstep. If the footstep doesn't have any predicted contact points, then use the default ones in the ContactablePlaneBodys.
      momentumBasedController.updateContactPointsForUpcomingFootstep(nextFootstep);
      momentumBasedController.updateBipedSupportPolygons();
   }

   @Override
   protected boolean hasMinimumTimePassed()
   {
      double minimumSwingTime;
      if (balanceManager.isRecoveringFromDoubleSupportFall())
         minimumSwingTime = 0.15;
      else
         minimumSwingTime = swingTime * minimumSwingFraction.getDoubleValue();

      return getTimeInCurrentState() > minimumSwingTime;
   }
}