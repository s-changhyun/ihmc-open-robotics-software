package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.testing.JUnitTools;

public class CompositeRigidBodyMassMatrixCalculatorTest extends MassMatrixCalculatorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testKineticEnergy()
   {
      setUpRandomChainRobot();
      double expectedKineticEnergy = computeKineticEnergy(joints);

      MassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(elevator);
      massMatrixCalculator.compute();
      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();
      double kineticEnergyFromMassMatrix = computeKineticEnergy(joints, massMatrix);

      assertEquals(expectedKineticEnergy, kineticEnergyFromMassMatrix, 1e-12);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSixDoFJoint()
   {
      Random random = new Random(1982L);
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoFJoint", elevator, elevator.getBodyFixedFrame());

      sixDoFJoint.setPositionAndRotation(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
      Twist sixDoFJointTwist = new Twist();
      sixDoFJoint.getJointTwist(sixDoFJointTwist);
      sixDoFJointTwist.setLinearPart(RandomGeometry.nextVector3D(random));
      sixDoFJointTwist.setAngularPart(RandomGeometry.nextVector3D(random));
      sixDoFJoint.setJointTwist(sixDoFJointTwist);

      RigidBody floating = ScrewTools.addRigidBody("floating", sixDoFJoint, RandomGeometry.nextDiagonalMatrix3D(random), random.nextDouble(),
            RandomGeometry.nextVector3D(random));
      MassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(elevator);
      massMatrixCalculator.compute();
      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();

      RigidBodyInertia inertia = floating.getInertiaCopy();
      inertia.changeFrame(ReferenceFrame.getWorldFrame());
      DenseMatrix64F inertiaMatrix = new DenseMatrix64F(sixDoFJoint.getDegreesOfFreedom(), sixDoFJoint.getDegreesOfFreedom());
      inertia.getMatrix(inertiaMatrix);

      JUnitTools.assertMatrixEquals(inertiaMatrix, massMatrix, 1e-5);
   }
}
