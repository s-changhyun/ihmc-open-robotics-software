package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.tools.io.printing.PrintTools;

public class ICPOptimizationControllerTest
{
   private final YoVariableRegistry registry = new YoVariableRegistry("robert");
   private final DoubleYoVariable omega = new DoubleYoVariable("omega", registry);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 0.0001;

   private static final double footLengthForControl = 0.25;
   private static final double footWidthForControl = 0.12;
   private static final double toeWidthForControl = 0.12;

   private static final double singleSupportDuration = 2.0;
   private static final double doubleSupportDuration = 1.0;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<FramePose> footPosesAtTouchdown = new SideDependentList<>(new FramePose(), new FramePose());
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   private BipedSupportPolygons bipedSupportPolygons;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStepping()
   {
      setupContactableFeet();
      setupBipedSupportPolygons();

      double omega0 = 0.3;
      omega.set(omega0);

      ICPPlanner icpPlanner = new ICPPlanner(bipedSupportPolygons, contactableFeet, icpPlannerParameters, registry, null);
      ICPOptimizationController icpOptimizationController = new ICPOptimizationController(icpPlannerParameters, icpOptimizationParameters, walkingControllerParameters, bipedSupportPolygons,
            contactableFeet, 0.001, registry, null);
      icpOptimizationController.setStepDurations(doubleSupportDuration, singleSupportDuration);
      icpPlanner.setOmega0(omega.getDoubleValue());

      icpPlanner.clearPlan();
      icpOptimizationController.clearPlan();

      double stepLength = 0.2;
      double stepWidth = 0.1;
      FootstepTestHelper footstepTestHelper = new FootstepTestHelper(contactableFeet, ankleFrames);
      List<Footstep> footsteps = footstepTestHelper.createFootsteps(stepWidth, stepLength, 3);
      FootstepTiming defaultTiming = new FootstepTiming(singleSupportDuration, doubleSupportDuration);
      icpPlanner.setFinalTransferTime(doubleSupportDuration);

      for (int i = 0; i < footsteps.size(); i++)
      {
         icpOptimizationController.addFootstepToPlan(footsteps.get(i));
         icpPlanner.addFootstepToPlan(footsteps.get(i), defaultTiming);
      }

      RobotSide supportSide = footsteps.get(0).getRobotSide().getOppositeSide();

      icpPlanner.setSupportLeg(supportSide);
      icpPlanner.initializeForSingleSupport(0.0);
      icpOptimizationController.initializeForSingleSupport(0.0, supportSide, omega0);

      icpPlanner.updateCurrentPlan();
      double currentTime = 0.5;
      FramePoint2d desiredICP = new FramePoint2d();
      FrameVector2d desiredICPVelocity = new FrameVector2d();
      FramePoint2d perfectCMP = new FramePoint2d();
      icpPlanner.getDesiredCapturePointPositionAndVelocity(desiredICP, desiredICPVelocity, currentTime);
      icpPlanner.getDesiredCentroidalMomentumPivotPosition(perfectCMP);
      icpOptimizationController.compute(currentTime, desiredICP, desiredICPVelocity, desiredICP, omega0);

      FramePoint2d desiredCMP = new FramePoint2d();
      icpOptimizationController.getDesiredCMP(desiredCMP);
      PrintTools.debug("Desired CMP = " + desiredCMP);
      PrintTools.debug("Perfect CMP = " + perfectCMP);
   }

   private void setupContactableFeet()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.084;

         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));

         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));
         contactableFoot.setSoleFrame(startingPose);

         contactableFeet.put(robotSide, contactableFoot);
      }
   }

   private void setupBipedSupportPolygons()
   {
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         RigidBody foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2d> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();

      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, ankleZUpFrames, registry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);
   }

   private static final ICPOptimizationParameters icpOptimizationParameters = new ICPOptimizationParameters()
   {
      @Override public int getMaximumNumberOfFootstepsToConsider()
      {
         return 5;
      }

      @Override public int numberOfFootstepsToConsider()
      {
         return 0;
      }

      @Override public double getForwardFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getLateralFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getFootstepRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackForwardWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackLateralWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackParallelGain()
      {
         return 2.0;
      }

      @Override public double getFeedbackOrthogonalGain()
      {
         return 3.0;
      }

      @Override public double getDynamicRelaxationWeight()
      {
         return 1000.0;
      }

      @Override public double getDynamicRelaxationDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override public boolean scaleStepRegularizationWeightWithTime()
      {
         return false;
      }

      @Override public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override public boolean scaleUpcomingStepWeights()
      {
         return false;
      }

      @Override public boolean useFeedbackRegularization()
      {
         return true;
      }

      @Override public boolean useStepAdjustment()
      {
         return true;
      }

      @Override public boolean useFootstepRegularization()
      {
         return true;
      }

      @Override public double getMinimumFootstepWeight()
      {
         return 0.0001;
      }

      @Override public double getMinimumFeedbackWeight()
      {
         return 0.0001;
      }

      @Override
      public double getMinimumTimeRemaining()
      {
         return 0.001;
      }

      @Override public double getDoubleSupportMaxCMPForwardExit()
      {
         return 0.05;
      }

      @Override public double getDoubleSupportMaxCMPLateralExit()
      {
         return 0.03;
      }

      @Override public double getSingleSupportMaxCMPForwardExit()
      {
         return 0.05;
      }

      @Override public double getSingleSupportMaxCMPLateralExit()
      {
         return 0.03;
      }

      @Override public double getAdjustmentDeadband()
      {
         return 0.0;
      }
   };

   private static final CapturePointPlannerParameters icpPlannerParameters = new CapturePointPlannerParameters()
   {
      @Override public double getEntryCMPInsideOffset()
      {
         return 0;
      }

      @Override public double getExitCMPInsideOffset()
      {
         return 0;
      }

      @Override public double getEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getExitCMPForwardOffset()
      {
         return 0;
      }

      @Override public boolean useTwoCMPsPerSupport()
      {
         return false;
      }

      @Override public double getMaxEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMinEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMaxExitCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMinExitCMPForwardOffset()
      {
         return 0;
      }
   };

   private static final WalkingControllerParameters walkingControllerParameters = new WalkingControllerParameters()
   {

      @Override
      public double getToeWidth()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getFootstepArea()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getFootWidth()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getFootLength()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getFootForwardOffset()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getFootBackwardOffset()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getActualFootWidth()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getActualFootLength()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getStepPitch()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMinStepWidth()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMinAreaPercentForValidFootstep()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxSwingHeightFromStanceFoot()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxStepWidth()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxStepUp()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxStepLength()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxStepDown()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxAngleTurnOutwards()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxAngleTurnInwards()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getInPlaceWidth()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getDesiredStepForward()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getDefaultStepLength()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getDangerAreaPercentForValidFootstep()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public boolean isNeckPositionControlled()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public double getTrajectoryTimeHeadOrientation()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getNeckPitchUpperLimit()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getNeckPitchLowerLimit()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double[] getInitialHeadYawPitchRoll()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public double getHeadYawLimit()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getHeadRollLimit()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public String[] getDefaultHeadOrientationControlJointNames()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoOrientationPIDGainsInterface createHeadOrientationControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoPIDGains createHeadJointspaceControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public void useVirtualModelControlCore()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public boolean useOptimizationBasedICPController()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public void useInverseDynamicsControlCore()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public double pelvisToAnkleThresholdForWalking()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double nominalHeightAboveAnkle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double minimumHeightAboveAnkle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double maximumHeightAboveAnkle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public boolean isSpinePitchReversed()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public double getToeTouchdownAngle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getTimeToGetPreparedForLocomotion()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getSwingHeightMaxForPushRecoveryTrajectory()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getSpineYawLimit()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getSpineRollLimit()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getSpinePitchUpperLimit()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getSpinePitchLowerLimit()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public double getSideLengthOfBoundingBoxForFootstepHeight()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getSecondContactThresholdForceIgnoringCoP()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getOmega0()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public double getMinimumSwingTimeForDisturbanceRecovery()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMinStepLengthForToeOff()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMinMechanicalLegLength()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMinLegLengthBeforeCollapsingSingleSupport()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaximumToeOffAngle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportY()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportX()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getLegLength()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public String[] getJointsToIgnoreInController()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public double getICPErrorThresholdToSpeedUpSwing()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getHighCoPDampingDurationToPreventFootShakies()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getHeelTouchdownAngle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getFoot_start_toetaper_from_back()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public FootSwitchType getFootSwitchType()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public double getDesiredTouchdownVelocity()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getDesiredTouchdownHeightOffset()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getDesiredTouchdownAcceleration()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public SideDependentList<RigidBodyTransform> getDesiredHandPosesWithRespectToChestFrame()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public double getDefaultTransferTime()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getDefaultSwingTime()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public String[] getDefaultChestOrientationControlJointNames()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public double getContactThresholdHeight()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getContactThresholdForce()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getCoPThresholdFraction()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getCoPErrorThresholdForHighCoPDamping()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public boolean getCoMHeightDriftCompensation()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public double getAnkleHeight()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public boolean finishSingleSupportWhenICPPlannerIsDone()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean doToeTouchdownIfPossible()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean doToeOffWhenHittingAnkleLimit()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean doToeOffIfPossibleInSingleSupport()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean doToeOffIfPossible()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean doPrepareManipulationForLocomotion()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean doHeelTouchdownIfPossible()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean doFancyOnToesControl()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public double defaultOffsetHeightAboveAnkle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public YoPDGains createUnconstrainedJointsControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoSE3PIDGainsInterface createToeOffFootControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoSE3PIDGainsInterface createSwingFootControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoOrientationPIDGainsInterface createPelvisOrientationControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoPDGains createPelvisICPBasedXYControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public ICPControlGains createICPControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoSE3PIDGainsInterface createHoldPositionFootControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoSE3PIDGainsInterface createEdgeTouchdownFootControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoOrientationPIDGainsInterface createChestControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public boolean controlHeadAndHandsWithSliders()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean checkECMPLocationToTriggerToeOff()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean allowShrinkingSingleSupportFootPolygon()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean allowAutomaticManipulationAbort()
      {
         // TODO Auto-generated method stub
         return false;
      }
   };
}
