package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;

public class AtlasJointPrivilegedConfigurationParameters extends JointPrivilegedConfigurationParameters
{
   private final boolean runningOnRealRobot;

   public AtlasJointPrivilegedConfigurationParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultConfigurationGain()
   {
      return 40.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultVelocityGain()
   {
      return 6.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultMaxVelocity()
   {
      return 2.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultMaxAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultWeight()
   {
      return 5.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getSupportKneeConfigurationGain()
   {
      return 100.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getSupportKneeVelocityGain()
   {
      return 6.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getUnconstrainedKneeConfigurationGain()
   {
      return 250.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getUnconstrainedKneeVelocityGain()
   {
      return 10.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getOnToesKneeConfigurationGain()
   {
      return 250.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getOnToesKneeVelocityGain()
   {
      return 10.0;
   }
}
