package us.ihmc.robotics.geometry.algorithms;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.shapes.FrameSphere3d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SphereWithConvexPolygonIntersectorTest
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSimpleIntersections()
   {
      SphereWithConvexPolygonIntersector sphereWithConvexPolygonIntersector = new SphereWithConvexPolygonIntersector();
      FrameSphere3d sphere;
      List<Point2D> vertices;
      FrameConvexPolygon2d polygon;
      PoseReferenceFrame frame;
      PoseReferenceFrame frame2;
      FramePoint closestPointOnPolygon;

      sphere = new FrameSphere3d(WORLD, 5.0, 3.0, 1.0, 2.0);
      vertices = new ArrayList<>();
      vertices.add(new Point2D(-10.0, -10.0));
      vertices.add(new Point2D(-10.0, 10.0));
      vertices.add(new Point2D(10.0, 10.0));
      vertices.add(new Point2D(10.0, -10.0));
      polygon = new FrameConvexPolygon2d(WORLD, vertices);
      assertTrue(sphereWithConvexPolygonIntersector.checkIfIntersectionExists(sphere, polygon));
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(5.0, 3.0, 0.0),
                                     sphereWithConvexPolygonIntersector.getClosestPointOnPolygon().getPoint(), Epsilons.ONE_TRILLIONTH);

      sphere = new FrameSphere3d(WORLD, 5.0, 3.0, 1.0, 0.5);
      vertices = new ArrayList<>();
      vertices.add(new Point2D(-10.0, -10.0));
      vertices.add(new Point2D(-10.0, 10.0));
      vertices.add(new Point2D(10.0, 10.0));
      vertices.add(new Point2D(10.0, -10.0));
      polygon = new FrameConvexPolygon2d(WORLD, vertices);
      assertFalse(sphereWithConvexPolygonIntersector.checkIfIntersectionExists(sphere, polygon));
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(5.0, 3.0, 0.0),
                                     sphereWithConvexPolygonIntersector.getClosestPointOnPolygon().getPoint(), Epsilons.ONE_TRILLIONTH);
      
      sphere = new FrameSphere3d(WORLD, 5.0, 3.0, 1.0, 0.5);
      vertices = new ArrayList<>();
      vertices.add(new Point2D(-1.0, -1.0));
      vertices.add(new Point2D(-1.0, 1.0));
      vertices.add(new Point2D(1.0, 1.0));
      vertices.add(new Point2D(1.0, -1.0));
      polygon = new FrameConvexPolygon2d(WORLD, vertices);
      assertFalse(sphereWithConvexPolygonIntersector.checkIfIntersectionExists(sphere, polygon));
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(1.0, 1.0, 0.0),
                                     sphereWithConvexPolygonIntersector.getClosestPointOnPolygon().getPoint(), Epsilons.ONE_TRILLIONTH);
      
      frame = new PoseReferenceFrame("testFrame1", WORLD);
      frame.setPositionWithoutChecksAndUpdate(5.0, 3.0, 0.0);
      sphere = new FrameSphere3d(WORLD, 5.0, 3.0, 1.0, 0.5);
      vertices = new ArrayList<>();
      vertices.add(new Point2D(-1.0, -1.0));
      vertices.add(new Point2D(-1.0, 1.0));
      vertices.add(new Point2D(1.0, 1.0));
      vertices.add(new Point2D(1.0, -1.0));
      polygon = new FrameConvexPolygon2d(frame, vertices);
      assertFalse(sphereWithConvexPolygonIntersector.checkIfIntersectionExists(sphere, polygon));
      closestPointOnPolygon = sphereWithConvexPolygonIntersector.getClosestPointOnPolygon();
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(5.0, 3.0, 0.0),
                                     closestPointOnPolygon.getPoint(), Epsilons.ONE_TRILLIONTH);
      closestPointOnPolygon.changeFrame(frame);
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(0.0, 0.0, 0.0),
                                     closestPointOnPolygon.getPoint(), Epsilons.ONE_TRILLIONTH);
      
      frame = new PoseReferenceFrame("testFrame2", WORLD);
      frame.setPositionWithoutChecksAndUpdate(5.0, 3.0, 0.0);
      sphere = new FrameSphere3d(WORLD, 5.0, 3.0, 1.0, 1.0);
      vertices = new ArrayList<>();
      vertices.add(new Point2D(-1.0, -1.0));
      vertices.add(new Point2D(-1.0, 1.0));
      vertices.add(new Point2D(1.0, 1.0));
      vertices.add(new Point2D(1.0, -1.0));
      polygon = new FrameConvexPolygon2d(frame, vertices);
      assertTrue(sphereWithConvexPolygonIntersector.checkIfIntersectionExists(sphere, polygon));
      closestPointOnPolygon = sphereWithConvexPolygonIntersector.getClosestPointOnPolygon();
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(5.0, 3.0, 0.0),
                                     closestPointOnPolygon.getPoint(), Epsilons.ONE_TRILLIONTH);
      closestPointOnPolygon.changeFrame(frame);
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(0.0, 0.0, 0.0),
                                     closestPointOnPolygon.getPoint(), Epsilons.ONE_TRILLIONTH);
      
      frame = new PoseReferenceFrame("testFrame3", WORLD);
      frame.setPositionWithoutChecksAndUpdate(4.0, 2.0, 0.0);
      sphere = new FrameSphere3d(WORLD, 5.0, 3.0, 1.0, 2.0);
      vertices = new ArrayList<>();
      vertices.add(new Point2D(-1.0, -1.0));
      vertices.add(new Point2D(-1.0, 1.0));
      vertices.add(new Point2D(1.0, 1.0));
      vertices.add(new Point2D(1.0, -1.0));
      polygon = new FrameConvexPolygon2d(frame, vertices);
      assertTrue(sphereWithConvexPolygonIntersector.checkIfIntersectionExists(sphere, polygon));
      closestPointOnPolygon = sphereWithConvexPolygonIntersector.getClosestPointOnPolygon();
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(5.0, 3.0, 0.0),
                                     closestPointOnPolygon.getPoint(), Epsilons.ONE_TRILLIONTH);
      closestPointOnPolygon.changeFrame(frame);
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(1.0, 1.0, 0.0),
                                     closestPointOnPolygon.getPoint(), Epsilons.ONE_TRILLIONTH);
      
      frame = new PoseReferenceFrame("testFrame4", WORLD);
      frame.setPositionWithoutChecksAndUpdate(4.0, 2.0, 0.0);
      sphere = new FrameSphere3d(frame, 5.0, 3.0, 1.0, 2.0);
      vertices = new ArrayList<>();
      vertices.add(new Point2D(-1.0, -1.0));
      vertices.add(new Point2D(-1.0, 1.0));
      vertices.add(new Point2D(1.0, 1.0));
      vertices.add(new Point2D(1.0, -1.0));
      frame2 = new PoseReferenceFrame("testFrame4", WORLD);
      frame2.setPositionWithoutChecksAndUpdate(9.5, 5.5, 0.0);
      polygon = new FrameConvexPolygon2d(frame2, vertices);
      assertTrue(sphereWithConvexPolygonIntersector.checkIfIntersectionExists(sphere, polygon));
      closestPointOnPolygon = sphereWithConvexPolygonIntersector.getClosestPointOnPolygon();
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(9.0, 5.0, 0.0),
                                     closestPointOnPolygon.getPoint(), Epsilons.ONE_TRILLIONTH);
      closestPointOnPolygon.changeFrame(frame2);
      EuclidCoreTestTools.assertTuple3DEquals("intersection not right", new Point3D(-0.5, -0.5, 0.0),
                                     closestPointOnPolygon.getPoint(), Epsilons.ONE_TRILLIONTH);
   }
}
