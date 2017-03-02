package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;

/** {@inheritDoc} */
public class AtlasICPOptimizationParameters extends ICPOptimizationParameters
{
   private final boolean runningOnRealRobot;

   public AtlasICPOptimizationParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   /** {@inheritDoc} */
   @Override
   public int numberOfFootstepsToConsider()
   {
      return 1;
   }

   /** {@inheritDoc} */
   @Override
   public double getForwardFootstepWeight()
   {
      return runningOnRealRobot ? 20.0 : 15.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getLateralFootstepWeight()
   {
      return runningOnRealRobot ? 20.0 : 15.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getFootstepRegularizationWeight()
   {
      return runningOnRealRobot ? 0.001 : 0.005;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackLateralWeight()
   {
      return runningOnRealRobot ? 0.5 : 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackForwardWeight()
   {
      return runningOnRealRobot ? 0.5 : 0.1;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackRegularizationWeight()
   {
      return runningOnRealRobot ? 0.0001 : 0.00005;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackParallelGain()
   {
      return runningOnRealRobot ? 3.0 : 3.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackOrthogonalGain()
   {
      return runningOnRealRobot ? 2.5 : 2.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getDynamicRelaxationWeight()
   {
      return runningOnRealRobot ? 500.0 : 1000.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDynamicRelaxationDoubleSupportWeightModifier()
   {
      return runningOnRealRobot ? 1.0 : 1.0;
   }

   /** {@inheritDoc} */
   @Override
   public boolean scaleStepRegularizationWeightWithTime()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public boolean scaleFeedbackWeightWithGain()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean scaleUpcomingStepWeights()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useFeedbackRegularization()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useStepAdjustment()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useFootstepRegularization()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFootstepWeight()
   {
      return 0.0001;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFeedbackWeight()
   {
      return 0.0001;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumTimeRemaining()
   {
      return 0.0001;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportMaxCMPForwardExit()
   {
      return 0.005;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportMaxCMPLateralExit()
   {
      return 0.005;
   }

   /** {@inheritDoc} */
   @Override
   public double getSingleSupportMaxCMPForwardExit()
   {
      return 0.005;
   }

   /** {@inheritDoc} */
   @Override
   public double getSingleSupportMaxCMPLateralExit()
   {
      return 0.005;
   }

   /** {@inheritDoc} */
   @Override
   public double getAdjustmentDeadband()
   {
      return 0.03;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportSplitFractionForBigAdjustment()
   {
      return runningOnRealRobot ? 0.25 : 0.2;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumTimeOnInitialCMPForBigAdjustment()
   {
      return runningOnRealRobot ? 0.15 : 0.1;
   }

   /** {@inheritDoc} */
   @Override
   public double getLateralReachabilityOuterLimit()
   {
      return 0.85;
   }

   /** {@inheritDoc} */
   @Override
   public double getLateralReachabilityInnerLimit()
   {
      return 0.18;
   }

   /** {@inheritDoc} */
   @Override
   public double getForwardReachabilityLimit()
   {
      return 0.9;
   }

   /** {@inheritDoc} */
   @Override
   public double getBackwardReachabilityLimit()
   {
      return -0.5;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useWarmStartInSolver()
   {
      return true;
   }
}
