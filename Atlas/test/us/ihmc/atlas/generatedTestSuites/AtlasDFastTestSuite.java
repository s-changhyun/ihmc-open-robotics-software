package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.testing.TestPlanSuite;
import us.ihmc.tools.testing.TestPlanSuite.TestSuiteTarget;
import us.ihmc.tools.testing.TestPlanTarget;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(TestPlanSuite.class)
@TestSuiteTarget(TestPlanTarget.Fast)
@SuiteClasses
({
   us.ihmc.atlas.behaviorTests.AtlasHeadOrientationBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasLookAtBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasObjectWeightBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasWalkToGoalBehaviorTest.class,
   us.ihmc.atlas.calib.KinematicCalibrationHeadLoopResidualTest.class,
   us.ihmc.atlas.commonWalkingControlModules.sensors.AtlasProvidedMassMatrixToolRigidBodyTest.class,
   us.ihmc.atlas.controllers.AtlasFootstepGeneratorTest.class,
   us.ihmc.atlas.controllers.responses.AtlasHandPoseStatusTest.class,
   us.ihmc.atlas.drcRobot.AtlasSDFVerificationTest.class,
   us.ihmc.atlas.driving.AtlasDrivingTest.class
})

public class AtlasDFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
