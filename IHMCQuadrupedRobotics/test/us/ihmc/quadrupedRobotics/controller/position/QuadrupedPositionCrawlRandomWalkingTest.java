package us.ihmc.quadrupedRobotics.controller.position;

import java.io.IOException;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedPositionTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedPositionCrawlRandomWalkingTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedPositionTestYoVariables variables;
   
   @Before
   public void setup()
   {
      try
      {
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
         ParameterRegistry.destroyAndRecreateInstance();
         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.POSITION);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedPositionTestYoVariables(conductor.getScs());
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }
   
   @After
   public void tearDown()
   {
      conductor = null;
      variables = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   private double randomValidVelocity(Random random)
   {
      return (2.0 * (random.nextDouble() - 0.5) * 0.15);
   }
   
   private double randomValidYawRate(Random random)
   {
      return (2.0 * (random.nextDouble() - 0.5) * 0.1);
   }
   
   private double randomSimulationDuration(Random random)
   {
      return random.nextDouble() * 10.0 + 5.0;
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 300.0)
   @Test(timeout = 1000000)
   public void testWalkingRandomly() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      Random random = new Random(1547L);
      double runningDuration = variables.getYoTime().getDoubleValue();
      
      for(int i = 0; i < 10; i++)
      {
         runningDuration += randomSimulationDuration(random);
         variables.getYoPlanarVelocityInputX().set(randomValidVelocity(random));
         variables.getYoPlanarVelocityInputZ().set(randomValidYawRate(random));
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
         
         runningDuration += 1.0;
         variables.getYoPlanarVelocityInputX().set(0.0);
         variables.getYoPlanarVelocityInputZ().set(0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
      }
      
      conductor.concludeTesting();
   }

   @ContinuousIntegrationTest(estimatedDuration = 200.0)
   @Test(timeout = 1000000)
   public void testWalkingAtRandomSpeedsWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      Random random = new Random(2456L);
      double runningDuration = variables.getYoTime().getDoubleValue();
      
      for(int i = 0; i < 6; i++)
      {
         double randomSimulationDuration = randomSimulationDuration(random);
         double randomValidVelocity = randomValidVelocity(random);
         runningDuration += randomSimulationDuration;
         variables.getYoPlanarVelocityInputX().set(randomValidVelocity);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
         
         runningDuration += 1.0;
         variables.getYoPlanarVelocityInputX().set(0.0);
         variables.getYoPlanarVelocityInputZ().set(0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
      }
      
      conductor.concludeTesting();
   }

   @ContinuousIntegrationTest(estimatedDuration = 300.0)
   @Test(timeout = 1000000)
   public void testWalkingRandomVelocitiesStoppingAndTurning() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      Random random = new Random(1557L);
      double runningDuration = variables.getYoTime().getDoubleValue();
      
      for(int i = 0; i < 6; i++)
      {
         double randomSimulationDuration = randomSimulationDuration(random);
         double randomValidVelocity = randomValidVelocity(random);
         runningDuration += randomSimulationDuration;
         variables.getYoPlanarVelocityInputX().set(randomValidVelocity);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
         
         runningDuration += 1.0;
         variables.getYoPlanarVelocityInputX().set(0.0);
         variables.getYoPlanarVelocityInputZ().set(0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
         
         randomSimulationDuration = randomSimulationDuration(random);
         double randomValidYawRate = randomValidYawRate(random);
         runningDuration += randomSimulationDuration;
         variables.getYoPlanarVelocityInputZ().set(randomValidYawRate);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
      }
      
      conductor.concludeTesting();
   }
}
