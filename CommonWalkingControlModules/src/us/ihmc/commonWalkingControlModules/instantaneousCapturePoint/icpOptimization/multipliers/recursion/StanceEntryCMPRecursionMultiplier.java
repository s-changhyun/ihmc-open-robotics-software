package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class StanceEntryCMPRecursionMultiplier
{
   private static final String name = "StanceCMPEntryRecursionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable entryMultiplier;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   public StanceEntryCMPRecursionMultiplier(String namePrefix, DoubleYoVariable exitCMPDurationInPercentOfStepTime,
         YoVariableRegistry parentRegistry)
   {
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

      entryMultiplier = new DoubleYoVariable(namePrefix + name, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      entryMultiplier.set(0.0);
   }

   public void compute(int numberOfFootstepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      if (numberOfFootstepsToConsider == 0)
      {
         this.entryMultiplier.set(0.0);
         return;
      }

      if (useTwoCMPs)
         computeWithTwoCMPs(doubleSupportDurations, singleSupportDurations, isInTransfer, omega0);
      else
         computeWithOneCMP(doubleSupportDurations, singleSupportDurations, omega0);
   }

   private void computeWithOneCMP(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double omega0)
   {
      double firstStepTime = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
      double entryMultiplier = 1.0 - Math.exp(-omega0 * firstStepTime);
      this.entryMultiplier.set(entryMultiplier);
   }

   private void computeWithTwoCMPs(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, boolean isInTransfer, double omega0)
   {
      double firstStepTime = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
      double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * firstStepTime;

      if (isInTransfer)
      {
         double entryMultiplier = 1.0 - Math.exp(-omega0 * timeSpentOnEntryCMP);
         this.entryMultiplier.set(entryMultiplier);
      }
      else
      {
         this.entryMultiplier.set(0.0);
      }
   }

   public double getEntryMultiplier()
   {
      return entryMultiplier.getDoubleValue();
   }
}
