package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class AdditionalSimulationContactPoints implements FootContactPoints
{
   private final int nContactPointsX;
   private final int nContactPointsY;
   private final boolean edgePointsOnly;

   private final boolean useSoftContactPointParameters;

   public AdditionalSimulationContactPoints(int nContactPointsX, int nContactPointsY, boolean edgePointsOnly, boolean useSoftContactPointParameters)
   {
      this.nContactPointsX = nContactPointsX;
      this.nContactPointsY = nContactPointsY;
      this.edgePointsOnly = edgePointsOnly;

      this.useSoftContactPointParameters = useSoftContactPointParameters;
   }

   @Override
   public Map<String, List<Tuple3DBasics>> getSimulationContactPoints(double footLength, double footWidth, double toeWidth, DRCRobotJointMap jointMap,
         SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms)
   {
      HashMap<String, List<Tuple3DBasics>> ret = new HashMap<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Tuple3DBasics> footContactPoints = new ArrayList<>();
         String parentJointName = jointMap.getJointBeforeFootName(robotSide);

         double dx = 1.01 * footLength / (nContactPointsX - 1.0);
         double xOffset = 1.01 * footLength / 2.0;

         for (int ix = 1; ix <= nContactPointsX; ix++)
         {
            double alpha = (ix - 1.0) / (nContactPointsX - 1.0);
            double footWidthAtCurrentX = (1.0 - alpha) * 1.01 * footWidth + alpha * 1.01 * toeWidth;
            double dy = footWidthAtCurrentX / (nContactPointsY - 1.0);
            double yOffset = footWidthAtCurrentX / 2.0;

            for (int iy = 1; iy <= nContactPointsY; iy++)
            {
               if (edgePointsOnly && ix != 1 && ix != nContactPointsX && iy != 1 && iy != nContactPointsY) // Only put points along the edges
                  continue;

               double x = (ix - 1) * dx - xOffset;
               double y = (iy - 1) * dy - yOffset;
               double z = 0.01 * ((xOffset - Math.abs(x))/xOffset + (yOffset - Math.abs(y))/yOffset);

               Point3D contactPoint = new Point3D(x, y, z);
               RigidBodyTransform transformToParentJointFrame = soleToAnkleFrameTransforms.get(robotSide);
               transformToParentJointFrame.transform(contactPoint);
               footContactPoints.add(contactPoint);
            }
         }

         ret.put(parentJointName, footContactPoints);
      }

      return ret;
   }

   @Override
   public SideDependentList<List<Tuple2DBasics>> getControllerContactPoints(double footLength, double footWidth, double toeWidth)
   {
      SideDependentList<List<Tuple2DBasics>> ret = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Tuple2DBasics> contactPoints = new ArrayList<>();
         contactPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
         contactPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
         contactPoints.add(new Point2D(footLength / 2.0, -toeWidth / 2.0));
         contactPoints.add(new Point2D(footLength / 2.0, toeWidth / 2.0));
         ret.put(robotSide, contactPoints);
      }

      return ret;
   }

   @Override
   public SideDependentList<Tuple2DBasics> getToeOffContactPoints(double footLength, double footWidth, double toeWidth)
   {
      SideDependentList<Tuple2DBasics> ret = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
         ret.put(robotSide, new Point2D(footLength / 2.0, 0.0));

      return ret;
   }

   @Override
   public boolean useSoftContactPointParameters()
   {
      return useSoftContactPointParameters;
   }

}
