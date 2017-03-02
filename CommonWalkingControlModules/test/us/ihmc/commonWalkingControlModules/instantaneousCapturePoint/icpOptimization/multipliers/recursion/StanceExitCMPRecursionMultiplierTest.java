package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class StanceExitCMPRecursionMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      StanceExitCMPRecursionMultiplier exitCMPRecursionMultiplier = new StanceExitCMPRecursionMultiplier("", exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         exitCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;
         double currentTimeSpentOnEntryCMP = (1.0 - exitRatio) * currentStepDuration;

         double exitCMPMultiplier = (1.0 - Math.exp(-omega * currentTimeSpentOnExitCMP));
         Assert.assertEquals(exitCMPMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         exitCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         exitCMPMultiplier = Math.exp(-omega * currentTimeSpentOnEntryCMP) * (1.0 - Math.exp(-omega * currentTimeSpentOnExitCMP));
         Assert.assertEquals(exitCMPMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStepOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      StanceExitCMPRecursionMultiplier exitCMPRecursionMultiplier = new StanceExitCMPRecursionMultiplier("", exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         exitCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         exitCMPRecursionMultiplier.compute(1, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoStepTwoCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      StanceExitCMPRecursionMultiplier exitCMPRecursionMultiplier = new StanceExitCMPRecursionMultiplier("", exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         exitCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         double currentStepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
         double currentTimeSpentOnExitCMP = exitRatio * currentStepDuration;
         double currentTimeSpentOnEntryCMP = (1.0 - exitRatio) * currentStepDuration;

         double exitCMPMultiplier = (1.0 - Math.exp(-omega * currentTimeSpentOnExitCMP));
         Assert.assertEquals(exitCMPMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         exitCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         exitCMPMultiplier = Math.exp(-omega * currentTimeSpentOnEntryCMP) * (1.0 - Math.exp(-omega * currentTimeSpentOnExitCMP));
         Assert.assertEquals(exitCMPMultiplier, exitCMPRecursionMultiplier.getExitMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoStepOneCMPCalculation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable yoOmega = new DoubleYoVariable("omega", registry);

      double omega = 3.0;
      yoOmega.set(omega);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      StanceExitCMPRecursionMultiplier exitCMPRecursionMultiplier = new StanceExitCMPRecursionMultiplier("", exitCMPRatio, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         exitCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(), epsilon);

         // setup for in transfer
         isInTransfer = true;
         exitCMPRecursionMultiplier.compute(2, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPRecursionMultiplier.getExitMultiplier(), epsilon);
      }
   }
}
