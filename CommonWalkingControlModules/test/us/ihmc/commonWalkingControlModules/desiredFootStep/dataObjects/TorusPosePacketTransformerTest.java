package us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.TorusPosePacket;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/14/13
 * Time: 10:14 AM
 * To change this template use File | Settings | File Templates.
 */
public class TorusPosePacketTransformerTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTransformTorusPosePacket()
   {
      int numberOfTests = 10;
      double radius = 1.0;

      Random random = new Random(100L);
      RigidBodyTransform transform3D;

      for (int i = 0; i < numberOfTests; i++)
      {
         Point3D point3d = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);

         AxisAngle axisAngle = RandomGeometry.nextAxisAngle(random);
         Quaternion quat = new Quaternion();
         quat.set(axisAngle);

         TorusPosePacket starting = new TorusPosePacket(point3d, quat, radius);

         transform3D = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         TorusPosePacket ending = starting.transform(transform3D);

         performEqualsTest(starting, transform3D, ending);
      }
   }

   private static void performEqualsTest(TorusPosePacket starting, RigidBodyTransform transform3D, TorusPosePacket ending)
   {
      // Point3D position;
      double distance = getDistanceBetweenPoints(starting.getPosition(), transform3D, ending.getPosition());
      assertEquals("not equal", 0.0, distance, 1e-6);

      Quaternion startQuat = starting.getOrientation();
      Quaternion endQuat = ending.getOrientation();
      assertTrue(areOrientationsEqualWithTransform(startQuat, transform3D, endQuat));
   }

   private static double getDistanceBetweenPoints(Point3D startingPoint, RigidBodyTransform transform3D, Point3D endPoint)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FramePoint start = new FramePoint(starting, startingPoint);
      FramePoint end = new FramePoint(ending, endPoint);

      end.changeFrame(starting);

      return end.distance(start);
   }

   private static boolean areOrientationsEqualWithTransform(Quaternion orientationStart, RigidBodyTransform transform3D, Quaternion orientationEnd)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FrameOrientation start = new FrameOrientation(starting, orientationStart);
      FrameOrientation end = new FrameOrientation(ending, orientationEnd);

      end.changeFrame(starting);

      return equalsFrameOrientation(start, end);
   }

   private static boolean equalsFrameOrientation(FrameOrientation frameOrientation1, FrameOrientation frameOrientation2)
   {
      // Check reference frame first
      if (frameOrientation1.getReferenceFrame() != frameOrientation2.getReferenceFrame())
         return false;

      double[] rpyThis = frameOrientation1.getYawPitchRoll();
      double[] rpyThat = frameOrientation2.getYawPitchRoll();

      for (int i = 0; i < rpyThat.length; i++)
      {
         if (Math.abs(rpyThis[i] - rpyThat[i]) > 1e-6)
            return false;
      }

      return true;
   }
}
