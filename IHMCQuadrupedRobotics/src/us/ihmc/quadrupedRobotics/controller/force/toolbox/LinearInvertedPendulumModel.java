package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class LinearInvertedPendulumModel
{
   private double mass;
   private double gravity;
   private double comHeight;
   private final ReferenceFrame comZUpFrame;

   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   DoubleYoVariable yoLipNaturalFrequency = new DoubleYoVariable("lipNaturalFrequency", registry);

   public LinearInvertedPendulumModel(ReferenceFrame comZUpFrame, double mass, double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      this.mass = mass;
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.comZUpFrame = comZUpFrame;
      parentRegistry.addChild(registry);
   }

   /**
    * @return natural frequency of the linear inverted pendulum
    */
   public double getNaturalFrequency()
   {
      return Math.sqrt(gravity / comHeight);
   }

   /**
    * @return time constant of the linear inverted pendulum
    */
   public double getTimeConstant()
   {
      return 1.0 / getNaturalFrequency();
   }

   /**
    * @param mass total mass of the robot
    */
   public void setMass(double mass)
   {
      this.mass = mass;
      this.yoLipNaturalFrequency.set(getNaturalFrequency());
   }

   /**
    * @return total mass of the robot
    */
   public double getMass()
   {
      return mass;
   }

   /**
    * @param gravity force of gravity in the vertical axis
    */
   public void setGravity(double gravity)
   {
      this.gravity = gravity;
      this.yoLipNaturalFrequency.set(getNaturalFrequency());
   }

   /**
    * @return force of gravity in the vertical axis
    */
   public double getGravity()
   {
      return gravity;
   }

   /**
    * @param comHeight nominal center of mass height relative to the eCMP
    */
   public void setComHeight(double comHeight)
   {
      this.comHeight = Math.max(comHeight, 0.001);
      this.yoLipNaturalFrequency.set(getNaturalFrequency());
   }

   /**
    * @return nominal center of mass height relative to the eCMP
    */
   public double getComHeight()
   {
      return comHeight;
   }

   /**
    * Compute the total contact force acting on the CoM given the desired eCMP.
    * @param comForce computed contact force acting on the center of mass
    * @param cmpPosition known position of the enhanced centroidal moment pivot
    */
   public void computeComForce(FrameVector comForce, FramePoint cmpPosition)
   {
      ReferenceFrame cmpPositionFrame = cmpPosition.getReferenceFrame();
      ReferenceFrame comForceFrame = comForce.getReferenceFrame();
      cmpPosition.changeFrame(comZUpFrame);
      comForce.changeFrame(comZUpFrame);

      double omega = getNaturalFrequency();
      comForce.set(cmpPosition);
      comForce.scale(-mass * Math.pow(omega, 2));
      comForce.changeFrame(comForceFrame);
      cmpPosition.changeFrame(cmpPositionFrame);
   }

   /**
    * Compute the eCMP position given the total contact force acting on the CoM.
    * @param cmpPosition computed position of the enhanced centroidal moment pivot
    * @param comForce known contact force acting on the center of mass
    */
   public void computeCmpPosition(FramePoint cmpPosition, FrameVector comForce)
   {
      ReferenceFrame cmpPositionFrame = cmpPosition.getReferenceFrame();
      ReferenceFrame comForceFrame = comForce.getReferenceFrame();
      cmpPosition.changeFrame(comZUpFrame);
      comForce.changeFrame(comZUpFrame);

      double omega = getNaturalFrequency();
      cmpPosition.set(comForce);
      cmpPosition.scale(-1.0 / (mass * Math.pow(omega, 2)));
      cmpPosition.changeFrame(cmpPositionFrame);
      comForce.changeFrame(comForceFrame);
   }
}