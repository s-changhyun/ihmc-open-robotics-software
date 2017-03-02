package us.ihmc.humanoidRobotics.footstep;

/**
 * Holds timings for the execution of a footstep.
 */
public class FootstepTiming
{
   private double swingTime = Double.NaN;
   private double transferTime = Double.NaN;
   private boolean hasAbsoluteTime = false;
   private double swingStartTime = Double.NaN;
   private double executionStartTime = Double.NaN;

   public FootstepTiming()
   {
   }

   public FootstepTiming(double swingTime, double transferTime)
   {
      setTimings(swingTime, transferTime);
   }

   /**
    * Sets the {@link #swingTime} and {@link #transferTime} of the footstep.
    */
   public void setTimings(double swingTime, double transferTime)
   {
      this.swingTime = swingTime;
      this.transferTime = transferTime;
   }

   /**
    * Returns the time from swing foot lift-off to touch-down.
    */
   public double getSwingTime()
   {
      return swingTime;
   }

   /**
    * Returns the time to transfer the weight to the stance foot before the step is taken.
    */
   public double getTransferTime()
   {
      return transferTime;
   }

   /**
    * Returns the sum of {@link #swingTime} and {@link #transferTime}. This is the total time the step takes from
    * beginning of transferring weight to the stance foot to the touch-down of the swing foot.
    */
   public double getStepTime()
   {
      return swingTime + transferTime;
   }

   /**
    * Returns true if the footstep has an absolute timing requirement.
    */
   public boolean hasAbsoluteTime()
   {
      return hasAbsoluteTime;
   }

   /**
    * Sets the timing requirements for this footstep. The swingStartTime is with respect to the executionStartTime.
    */
   public void setAbsoluteTime(double swingStartTime, double executionStartTime)
   {
      hasAbsoluteTime = true;
      this.swingStartTime = swingStartTime;
      this.executionStartTime = executionStartTime;
   }

   /**
    * Removed all absolute timing requirements of this footstep.
    */
   public void removeAbsoluteTime()
   {
      hasAbsoluteTime = false;
      this.swingStartTime = Double.NaN;
      this.executionStartTime = Double.NaN;
   }

   /**
    * If absolute footstep timing is requested this will return the time at which the swing needs to start with respect
    * to the start time of the execution of the footstep plan that this footstep is part of.
    */
   public double getSwingStartTime()
   {
      return swingStartTime;
   }

   /**
    * This returns the time at which the footstep plan that this footstep is part of was started to execute. The value
    * is the reference for absolute footstep timings.
    */
   public double getExecutionStartTime()
   {
      return executionStartTime;
   }

   /**
    * Sets this timing to be a copy of the given one.
    */
   public void set(FootstepTiming other)
   {
      swingTime = other.swingTime;
      transferTime = other.transferTime;
      hasAbsoluteTime = other.hasAbsoluteTime;
      swingStartTime = other.swingStartTime;
      executionStartTime = other.executionStartTime;
   }
}
