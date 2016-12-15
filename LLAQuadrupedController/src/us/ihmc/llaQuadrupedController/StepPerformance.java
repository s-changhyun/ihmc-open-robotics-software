package us.ihmc.llaQuadrupedController;

import java.io.IOException;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;

public class StepPerformance
{
   public static void main(String[] args) throws IOException
   {
      QuadrupedForceControllerState states[] = { QuadrupedForceControllerState.STEP };
      LLAQuadrupedControllerFactoryDummyOutputDemo llaQuadrupedControllerFactoryDummyOutputDemo = new LLAQuadrupedControllerFactoryDummyOutputDemo(states);
      
      llaQuadrupedControllerFactoryDummyOutputDemo.run();
      
      System.exit(0);
   }
}
