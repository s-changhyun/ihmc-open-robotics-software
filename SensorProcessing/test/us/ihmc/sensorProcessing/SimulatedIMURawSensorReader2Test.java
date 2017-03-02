package us.ihmc.sensorProcessing;

import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.simulatedSensors.PerfectSimulatedIMURawSensorReader;
import us.ihmc.simulationconstructionset.simulatedSensors.SimulatedIMURawSensorReader;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;



public class SimulatedIMURawSensorReader2Test
{
   private Random random;
   
   @Before
   public void setUp()
   {
      random = new Random(1776L);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void test() throws SimulationExceededMaximumTimeException, UnreasonableAccelerationException
   {
      SingleRigidBodyRobot robot = new SingleRigidBodyRobot();
      robot.setPosition(RandomGeometry.nextVector3D(random));
      robot.setYawPitchRoll(random.nextDouble(), random.nextDouble(), random.nextDouble());
      robot.setAngularVelocity(RandomGeometry.nextVector3D(random));
      robot.setLinearVelocity(RandomGeometry.nextVector3D(random));
      robot.setExternalForce(RandomGeometry.nextVector3D(random));
      
      SimulatedSensorsTestFullRobotModel fullRobotModel = new SimulatedSensorsTestFullRobotModel();
      YoVariableRegistry registry = new YoVariableRegistry("test");
      RawSensors rawSensors = new RawSensors(registry);
      ReferenceFrame imuFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
      ProcessedSensorsReadWrite processedSensors = new ProcessedSensorsReadWrite(imuFrame, registry);
      int imuIndex = 0;
      RigidBody rigidBody = fullRobotModel.getBodyLink();
      RigidBody rootBody = fullRobotModel.getElevator();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), rootBody.getBodyFixedFrame());
      SimulatedIMURawSensorReader simulatedIMURawSensorReader = new PerfectSimulatedIMURawSensorReader(rawSensors, imuIndex, rigidBody, imuFrame, rootBody, rootAcceleration);
      PerfectIMUSensorProcessor imuSensorProcessor = new PerfectIMUSensorProcessor(rawSensors, processedSensors);

      simulatedIMURawSensorReader.initialize();
      imuSensorProcessor.initialize();
      robot.doDynamicsButDoNotIntegrate();
      fullRobotModel.update(robot);
      simulatedIMURawSensorReader.read();
      imuSensorProcessor.update();

      Vector3D linearAccelerationFromRobot = robot.getBodyAcceleration().getVectorCopy();
      Vector3D linearAccelerationFromIMU = processedSensors.getAcceleration(imuIndex).getVectorCopy();

//      System.out.println("linear from robot: " + linearAccelerationFromRobot);
//      System.out.println("linear from imu: " + linearAccelerationFromIMU);
      
      EuclidCoreTestTools.assertTuple3DEquals(linearAccelerationFromRobot, linearAccelerationFromIMU, 1e-9);
   }
}
