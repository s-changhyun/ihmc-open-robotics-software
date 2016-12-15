package us.ihmc.atlas.commonWalkingControlModules;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.ICPStraightLegWalkingTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasICPStraightLegWalkingTest extends ICPStraightLegWalkingTest
{
   protected DRCRobotModel getRobotModel()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
      return atlasRobotModel;
   }

   public static void main(String[] args)
   {
      AtlasICPStraightLegWalkingTest test = new AtlasICPStraightLegWalkingTest();
      try
      {
         test.testWalking();
      }
      catch(SimulationExceededMaximumTimeException e)
      {

      }
   }
}
