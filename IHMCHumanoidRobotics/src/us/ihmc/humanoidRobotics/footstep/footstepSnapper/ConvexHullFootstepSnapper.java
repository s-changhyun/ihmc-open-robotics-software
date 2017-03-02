package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.List;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.HullFace;
import us.ihmc.robotics.geometry.InsufficientDataException;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.geometry.PlaneFitter;
import us.ihmc.robotics.geometry.QuickHull3dWrapper;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * Created by agrabertilton on 1/14/15.
 */
public class ConvexHullFootstepSnapper implements FootstepSnapper
{
   public class PointWriter
   {
      private File outputFile;
      private int index = 0;

      public PointWriter(String basicName)
      {
         String filename = "FootstepPointsLists" + File.separator;
         filename += basicName + new Date().getTime() + ".txt";
         outputFile = new File(filename);
      }

      public void writeFootstepAndPointsToFile(FootstepDataMessage footstep, List<Point3D> points)
      {
         System.out.println("Printing Footstep and Points to file: " + outputFile.getName());
         index++;

         try
         {
            FileWriter writer = new FileWriter(outputFile, true);
            writer.write("Footstep " + index + "\n" + footstep + "\n");

            for (Point3D point3d : points)
            {
               writer.write(point3d.toString() + "\n");
            }

            writer.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   private static final boolean RECORD_BAD_FOOTSTEPS = false;

   // Generates Footsteps using grid without a mask
   private List<Point3D> pointList = new ArrayList<Point3D>();
   private final PlaneFitter planeFitter = new LeastSquaresZPlaneFitter();
   private FootstepValueFunction footstepValueFunction;
   private FootstepSnappingParameters parameters;
   private BasicFootstepMask footstepMask = null;
   private final SimpleFootstepSnapper simpleSnapper = new SimpleFootstepSnapper();
   private boolean useMask = false;
   private double maskBuffer = 0.0;
   private static PointWriter writer = null;

   public ConvexHullFootstepSnapper(FootstepValueFunction valueFunction, FootstepSnappingParameters parameters)
   {
      this.footstepValueFunction = valueFunction;
      this.parameters = parameters;
      this.useMask = true;
      this.footstepMask = new BasicFootstepMask(parameters.getCollisionPolygon(), maskBuffer);
      if (writer == null)
         writer = new PointWriter("DataFromConvexHullSnapper");
   }

   @Override
   public void setMask(List<Point2D> footShape)
   {
      this.footstepMask = new BasicFootstepMask(footShape, maskBuffer);
   }

   @Override
   public void setUseMask(boolean useMask, double maskSafetyBuffer, double boundingBoxDimension)
   {
      this.useMask = useMask;
      maskBuffer = maskSafetyBuffer;

      this.footstepMask = new BasicFootstepMask(parameters.getCollisionPolygon(), maskBuffer);

   }

   public void updateParameters(FootstepSnappingParameters newParameters)
   {
      parameters.updateParameters(newParameters);
      footstepValueFunction.updateFunction();

      this.footstepMask = new BasicFootstepMask(parameters.getCollisionPolygon(), maskBuffer);
   }

   @Override
   public List<Point3D> getPointList()
   {
      return pointList;
   }

   public FootstepSnappingParameters getParameters()
   {
      return parameters;
   }

   @Override
   public void adjustFootstepWithoutHeightmap(FootstepDataMessage footstep, double height, Vector3D planeNormal)
   {
      simpleSnapper.adjustFootstepWithoutHeightmap(footstep, height, planeNormal);
   }

   @Override
   public void adjustFootstepWithoutHeightmap(Footstep footstep, double height, Vector3D planeNormal)
   {
      simpleSnapper.adjustFootstepWithoutHeightmap(footstep, height, planeNormal);
   }

   @Override
   public Footstep generateFootstepWithoutHeightMap(FramePose2d footPose2d, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide, double height,
         Vector3D planeNormal)
   {
      return simpleSnapper.generateFootstepWithoutHeightMap(footPose2d, foot, soleFrame, robotSide, height, planeNormal);
   }

   @Override
   public Footstep generateSnappedFootstep(double soleX, double soleY, double yaw, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide,
         HeightMapWithPoints heightMap) throws InsufficientDataException
   {
      FramePose2d footPose2d = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(soleX, soleY), yaw);

      return generateFootstepUsingHeightMap(footPose2d, foot, soleFrame, robotSide, heightMap);
   }

   public Footstep generateFootstepUsingHeightMap(FramePose2d footPose2d, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide,
         List<Point3D> pointList, double defaultHeight)
   {
      Footstep toReturn = generateFootstepWithoutHeightMap(footPose2d, foot, soleFrame, robotSide, 0.0, new Vector3D(0.0, 0.0, 1.0));
      snapFootstep(toReturn, pointList, defaultHeight);

      return toReturn;
   }

   @Override
   public Footstep generateFootstepUsingHeightMap(FramePose2d footPose2d, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide,
         HeightMapWithPoints heightMap) throws InsufficientDataException
   {
      Footstep toReturn = generateFootstepWithoutHeightMap(footPose2d, foot, soleFrame, robotSide, 0.0, new Vector3D(0.0, 0.0, 1.0));
      snapFootstep(toReturn, heightMap);

      return toReturn;
   }

   @Override
   public Footstep.FootstepType snapFootstep(Footstep footstep, HeightMapWithPoints heightMap)
   {
      FootstepDataMessage originalFootstep = new FootstepDataMessage(footstep);

      //set to the sole pose
      Vector3D position = new Vector3D();
      Quaternion orientation = new Quaternion();
      RigidBodyTransform solePose = new RigidBodyTransform();
      footstep.getSolePose(solePose);
      solePose.get(orientation, position);
      originalFootstep.setLocation(new Point3D(position));
      originalFootstep.setOrientation(orientation);

      //get the footstep
      Footstep.FootstepType type = snapFootstep(originalFootstep, heightMap);
      if (type == Footstep.FootstepType.FULL_FOOTSTEP && originalFootstep.getPredictedContactPoints() != null)
      {
         throw new RuntimeException(this.getClass().getSimpleName() + "Full Footstep should have null contact points");
      }
      footstep.setPredictedContactPointsFromPoint2ds(originalFootstep.getPredictedContactPoints());
      footstep.setFootstepType(type);
      FramePose solePoseInWorld = new FramePose(ReferenceFrame.getWorldFrame(), originalFootstep.getLocation(), originalFootstep.getOrientation());
      footstep.setSolePose(solePoseInWorld);

      footstep.setSwingHeight(originalFootstep.getSwingHeight());
      footstep.setTrajectoryType(originalFootstep.getTrajectoryType());

      return type;
   }

   @Override
   public Footstep.FootstepType snapFootstep(FootstepDataMessage footstep, HeightMapWithPoints heightMap)
   {
      FootstepDataMessage originalFootstepFound = new FootstepDataMessage(footstep);
      Point3D position = originalFootstepFound.getLocation();
      double yaw = originalFootstepFound.getOrientation().getYaw();

      if (!useMask)
      {
         pointList = heightMap.getAllPointsWithinArea(position.getX(), position.getY(), parameters.getBoundingSquareSizeLength(),
               parameters.getBoundingSquareSizeLength());
      }
      else
      {
         footstepMask.setPositionAndYaw(position.getX(), position.getY(), yaw);
         pointList = heightMap.getAllPointsWithinArea(position.getX(), position.getY(), parameters.getBoundingSquareSizeLength(),
               parameters.getBoundingSquareSizeLength(), footstepMask);
      }

      double height = heightMap.getHeightAtPoint(position.getX(), position.getY());
      return snapFootstep(footstep, pointList, height);
   }

   public Footstep.FootstepType snapFootstep(Footstep footstep, List<Point3D> pointList, double defaultHeight)
   {
      FootstepDataMessage originalFootstep = new FootstepDataMessage(footstep);

      //set to the sole pose
      Vector3D position = new Vector3D();
      Quaternion orientation = new Quaternion();
      RigidBodyTransform solePose = new RigidBodyTransform();
      footstep.getSolePose(solePose);
      solePose.get(orientation, position);
      originalFootstep.setLocation(new Point3D(position));
      originalFootstep.setOrientation(orientation);

      //get the footstep
      Footstep.FootstepType type = snapFootstep(originalFootstep, pointList, defaultHeight);
      footstep.setFootstepType(type);
      footstep.setPredictedContactPointsFromPoint2ds(originalFootstep.getPredictedContactPoints());
      FramePose solePoseInWorld = new FramePose(ReferenceFrame.getWorldFrame(), originalFootstep.getLocation(), originalFootstep.getOrientation());
      footstep.setSolePose(solePoseInWorld);

      footstep.setSwingHeight(originalFootstep.getSwingHeight());
      footstep.setTrajectoryType(originalFootstep.getTrajectoryType());

      return type;
   }

   public Footstep.FootstepType snapFootstep(FootstepDataMessage footstep, List<Point3D> pointList, double defaultHeight)
   {
      Point3D position = footstep.getLocation();
      Quaternion orientation = footstep.getOrientation();

      // bad list of points
      double height;
      Vector3D surfaceNormal = new Vector3D();
      if (pointList == null || pointList.isEmpty())
      {
         height = defaultHeight;
         surfaceNormal.set(0.0, 0.0, 1.0);

         if (Double.isInfinite(height) || Double.isNaN(height))
         {
            height = 0.0;
         }

         adjustFootstepWithoutHeightmap(footstep, height, surfaceNormal);
         return Footstep.FootstepType.BAD_FOOTSTEP;
      }

      // good list of points, fit plane;
      Plane3d fittedPlane = new Plane3d();
      double avgSquaredError = planeFitter.fitPlaneToPoints(new Point2D(position.getX(), position.getY()), pointList, fittedPlane);
      height = fittedPlane.getZOnPlane(position.getX(), position.getY());

      surfaceNormal = fittedPlane.getNormalCopy();

      // check point fit to plane.
      int notOnPlaneCount = 0;
      boolean badPlane = false;

      int badnumberOfPointsthreshold = parameters.getBadnumberOfPointsthreshold();
      double tolerance = parameters.getZDistanceTolerance();
      double errorThreshold = 1.0;

      if ((avgSquaredError > errorThreshold) || surfaceNormal.containsNaN())
      {
         badPlane = true;
      }
      else
      {
         // check the plane
         for (Point3D point : pointList)
         {
            if (Math.abs(fittedPlane.getZOnPlane(point.getX(), point.getY()) - point.getZ()) > tolerance)
            {
               notOnPlaneCount++;

               if (notOnPlaneCount >= badnumberOfPointsthreshold)
               {
                  badPlane = true;

                  break;
               }
            }
         }
      }

      // good plane, return plane
      if (!badPlane)
      {
         adjustFootstepWithoutHeightmap(footstep, height, surfaceNormal);
         footstep.predictedContactPoints = null;
         return Footstep.FootstepType.FULL_FOOTSTEP;
      }

      // bad plane, try convex hull fit
      boolean footstepFound = computePartialFootstepFromPoints(footstep, pointList);

      // good footstep
      if (footstepFound)
      {
         return Footstep.FootstepType.PARTIAL_FOOTSTEP;
      }

      // bad footstep
      // couldn't find a good footstep, footstep to max height found inside mask
      surfaceNormal.set(0.0, 0.0, 1.0);
      height = Double.NEGATIVE_INFINITY;

      for (Point3D point : pointList)
      {
         if ((Double.isInfinite(height) || Double.isNaN(height))
               || ((point.getZ() > height) && !Double.isInfinite(point.getZ()) && !Double.isNaN(point.getZ())))
         {
            height = point.getZ();
         }
      }

      if (Double.isNaN(height) || Double.isInfinite(height))
         height = 0.0;

      adjustFootstepWithoutHeightmap(footstep, height, surfaceNormal);
      return Footstep.FootstepType.BAD_FOOTSTEP;
   }

   private boolean computePartialFootstepFromPoints(FootstepDataMessage footstep, List<Point3D> points)
   {
      // check input to see if everything is infinite
      List<Point3D> convexHullPointsList = new ArrayList<Point3D>();

      boolean anInfinitePoint = false;
      for (Point3D point : points)
      {
         if (Double.isInfinite(point.getZ()))
         {
            anInfinitePoint = true;

            break;
         }

         convexHullPointsList.add(point);
      }

      if (anInfinitePoint)
         return false;

      // add in lower boundary points
      Point3D position = footstep.getLocation();
      Quaternion orientation = footstep.getOrientation();
      double yaw = orientation.getYaw();
      addLowerBoundaryPointsToHullPointList(convexHullPointsList, position.getX(), position.getY(), yaw);

      // Convert from points to possible support polygons
      QuickHull3dWrapper quickHull = new QuickHull3dWrapper();
      try
      {
         quickHull.build(convexHullPointsList);
      }
      catch (Exception e)
      {
         System.out.println("Illegal arguement to convex hull");

         throw new RuntimeException(e);
      }

      List<HullFace> faces = quickHull.getFaces();

      //    double distanceTolerance = quickHull.getDistanceTolerance();
      double distanceTolerance = parameters.getZDistanceTolerance();

      // get the orientations and heights corresponding to those faces
      // for each footstep, calculate the support polygon out of the points near it
      List<FootstepDataMessage> possibleFootsteps = new ArrayList<FootstepDataMessage>();
      Plane3d facePlane = new Plane3d();
      Vector3D planeNormal = new Vector3D();
      double x = position.getX();
      double y = position.getY();
      double maxValue = Double.NEGATIVE_INFINITY;
      FootstepDataMessage maxValueFootstep = null;
      double valueOfCurrent;

      ArrayList<? extends Point2DReadOnly> currentPredictedContactPoints;
      for (HullFace face : faces)
      {
         if (face.getSlopeAngle() > Math.PI / 4)
         {
            continue;
         }

         face.getPlane(facePlane);
         Quaternion newOrientation = new Quaternion();
         RotationTools.computeQuaternionFromYawAndZNormal(yaw, facePlane.getNormalCopy(), newOrientation);
         FootstepDataMessage currentFaceFootstep = new FootstepDataMessage(footstep.getRobotSide(), new Point3D(x, y, facePlane.getZOnPlane(x, y)),
               newOrientation);
         currentPredictedContactPoints = getPredictedContactPointsForFootstep(currentFaceFootstep, points, distanceTolerance);

         if ((currentPredictedContactPoints == null) || (currentPredictedContactPoints.size() < 3))
         {
            continue;
         }

         currentFaceFootstep.setPredictedContactPoints((ArrayList<Point2D>) currentPredictedContactPoints);
         valueOfCurrent = footstepValueFunction.getFootstepValue(currentFaceFootstep);

         if (valueOfCurrent > maxValue)
         {
            maxValue = valueOfCurrent;
            maxValueFootstep = currentFaceFootstep;
         }
      }

      if (maxValueFootstep == null)
      {
         if (RECORD_BAD_FOOTSTEPS)
            writer.writeFootstepAndPointsToFile(footstep, pointList);

         return false;
      }

      // determine the footstep with the highest value, then
      footstep.setLocation(maxValueFootstep.getLocation());
      footstep.setOrientation(maxValueFootstep.getOrientation());
      footstep.setPredictedContactPoints(maxValueFootstep.getPredictedContactPoints());
      return true;
   }

   private void addLowerBoundaryPointsToHullPointList(List<Point3D> convexHullPointsList, FramePose2d positioning)
   {
      addLowerBoundaryPointsToHullPointList(convexHullPointsList, positioning.getX(), positioning.getY(), positioning.getYaw());
   }

   private void addLowerBoundaryPointsToHullPointList(List<Point3D> convexHullPointsList, double xOrigin, double yOrigin, double yaw)
   {
      double minHeight = Double.POSITIVE_INFINITY;
      for (Point3D point : convexHullPointsList)
      {
         if (point.getZ() < minHeight)
            minHeight = point.getZ();
      }

      ConvexPolygon2d lowerBoundPolygon = parameters.getCollisionPolygon();
      double dropDistance = parameters.getBoundingSquareSizeLength();
      double cosYaw = Math.cos(yaw);
      double sinYaw = Math.sin(yaw);

      int numVertices = lowerBoundPolygon.getNumberOfVertices();
      Point2DReadOnly vertex;
      double xCoord;
      double yCoord;
      for (int i = 0; i < numVertices; i++)
      {
         vertex = lowerBoundPolygon.getVertex(i);
         xCoord = xOrigin + cosYaw * vertex.getX() - sinYaw * vertex.getY();
         yCoord = yOrigin + cosYaw * vertex.getY() + sinYaw * vertex.getX();
         convexHullPointsList.add(new Point3D(xCoord, yCoord, minHeight - dropDistance));
      }
   }

   private ArrayList<? extends Point2DReadOnly> getPredictedContactPointsForFootstep(FootstepDataMessage footstepData, List<Point3D> points, double distanceTolerance)
   {
      // get the plane of the footstep
      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.set(footstepData.getOrientation());
      Vector3D footNormal = new Vector3D();
      rotationMatrix.getColumn(2, footNormal);
      Plane3d footPlane = new Plane3d(footstepData.getLocation(), footNormal);

      // get all points within distanceTolerance of the plane
      List<Point3D> pointsNearPlane = new ArrayList<Point3D>();
      for (Point3D point : points)
      {
         if (footPlane.distance(point) <= distanceTolerance)
         {
            pointsNearPlane.add(point);
         }
      }

      // transform all the points into the footstep plane frame
      List<Point2D> pointsInFootstepFrame = new ArrayList<Point2D>();
      Point3D tempPoint = new Point3D();
      rotationMatrix.transpose();

      for (Point3D point : pointsNearPlane)
      {
         tempPoint.set(point);
         tempPoint.setZ(footPlane.getZOnPlane(tempPoint.getX(), tempPoint.getY()));
         tempPoint.sub(footstepData.getLocation());
         rotationMatrix.transform(tempPoint);

         if (Math.abs(tempPoint.getZ()) > 1e-14)
         {
            System.out.println("Error in ComplexFootstepSnapper");
         }

         pointsInFootstepFrame.add(new Point2D(tempPoint.getX(), tempPoint.getY()));
      }

      // find the convex hull of all the points inside the supportPolgon convex hull
      ConvexPolygon2d maxSupportPolygon = parameters.getSupportPolygon();
      ConvexPolygon2d actualSupportPolygon = new ConvexPolygon2d();
      for (Point2D point2d : pointsInFootstepFrame)
      {
         if (maxSupportPolygon.isPointInside(point2d, distanceTolerance))
         {
            actualSupportPolygon.addVertex(point2d);
         }
      }

      actualSupportPolygon.update();

      // crop the end vertices
      ArrayList<Point2DReadOnly> supportPoints = new ArrayList<>();
      int numPoints = actualSupportPolygon.getNumberOfVertices();
      for (int i = 0; i < numPoints; i++)
      {
         supportPoints.add(actualSupportPolygon.getVertex(i));
      }

      int cropNumber = 4;
      ArrayList<? extends Point2DReadOnly> finalSupportPoints;
      if (cropNumber == 4)
      {
         finalSupportPoints = reduceListOfPointsToFourFootstepBased(supportPoints);
         //         finalSupportPoints = reduceListOfPointsByArea(supportPoints, cropNumber);
      }
      else
      {
         finalSupportPoints = reduceListOfPointsByArea(supportPoints, cropNumber);
      }

      if (finalSupportPoints == null || finalSupportPoints.isEmpty())
         return null;

      return finalSupportPoints;
   }

   // class to help with cropping calculations
   public class VertexData implements Comparable<VertexData>
   {
      Point2DReadOnly position;
      double area;
      double distanceToCentroid;
      VertexData nextVertexData;
      VertexData previousVertexData;

      public VertexData(Point2DReadOnly position)
      {
         this.position = position;
         this.area = 0.0;
         this.distanceToCentroid = 0.0;
      }

      @Override
      public int compareTo(VertexData vertexData)
      {
         if (distanceToCentroid < vertexData.distanceToCentroid)
         {
            return -1;
         }

         if (distanceToCentroid > vertexData.distanceToCentroid)
         {
            return 1;
         }

         return 0;
      }

      public void remove()
      {
         // update the other references
         nextVertexData.previousVertexData = this.previousVertexData;
         previousVertexData.nextVertexData = this.nextVertexData;
         nextVertexData.calculateArea();
         previousVertexData.calculateArea();

      }

      public void calculateArea()
      {
         Point2D forwardVector = new Point2D(nextVertexData.position);
         forwardVector.sub(position);

         Point2D backwardsVector = new Point2D(previousVertexData.position);
         backwardsVector.sub(position);

         area = Math.abs(forwardVector.getX() * backwardsVector.getY() - forwardVector.getY() * backwardsVector.getX());
      }
   }

   public class VertexAreaComparator implements Comparator<VertexData>
   {
      @Override
      public int compare(VertexData o1, VertexData o2)
      {
         if (o1.area == o2.area)
            return 0;
         if (o1.area < o2.area)
            return -1;

         return 1;
      }
   }

   public ArrayList<Point2DReadOnly> reduceListOfPointsByArea(List<? extends Point2DReadOnly> listOfPoints, int maxNumPoints)
   {
      ConvexPolygon2d supportPolygon = new ConvexPolygon2d(listOfPoints);
      supportPolygon.update();
      Point2DReadOnly polygonCentroid = supportPolygon.getCentroid();

      int currentNumberOfVertices = supportPolygon.getNumberOfVertices();
      ArrayList<VertexData> verticesOfPolygon = new ArrayList<VertexData>();

      for (int i = 0; i < currentNumberOfVertices; i++)
      {
         verticesOfPolygon.add(new VertexData(supportPolygon.getVertex(i)));
      }

      for (int i = 0; i < currentNumberOfVertices; i++)
      {
         VertexData currentVertex = verticesOfPolygon.get(i);
         currentVertex.distanceToCentroid = polygonCentroid.distance(currentVertex.position);
         currentVertex.previousVertexData = verticesOfPolygon.get((i - 1 + currentNumberOfVertices) % currentNumberOfVertices);
         currentVertex.nextVertexData = verticesOfPolygon.get((i + 1) % currentNumberOfVertices);
         currentVertex.calculateArea();
      }

      Comparator<VertexData> areaComparator = new VertexAreaComparator();
      while (currentNumberOfVertices > maxNumPoints)
      {
         Collections.sort(verticesOfPolygon, areaComparator);
         VertexData minAreaVertexData = verticesOfPolygon.remove(0);
         minAreaVertexData.remove();
         currentNumberOfVertices--;
      }

      ArrayList<Point2DReadOnly> finalListOfSupportPoints = new ArrayList<>();
      for (VertexData vertex : verticesOfPolygon)
      {
         finalListOfSupportPoints.add(vertex.position);
      }

      return finalListOfSupportPoints;
   }

   private ArrayList<? extends Point2DReadOnly> reduceListOfPointsToFourFootstepBased(List<? extends Point2DReadOnly> listOfPoints)
   {

      ConvexPolygon2d basePolygon = parameters.getCollisionPolygon();
      ConvexPolygon2d supportPolygon = new ConvexPolygon2d(listOfPoints);
      supportPolygon.update();

      ArrayList<Point2D> finalListOfSupportPoints = new ArrayList<Point2D>();
      //for each vertex of the basePolygon, find the closest point inside the support polygon.
      int size = basePolygon.getNumberOfVertices();
      for (int i = 0; i < size; i++)
      {
         Point2DReadOnly vertex = basePolygon.getVertex(i);
         Point2D correspondingSupportPoint = getPointInPolygonNearestPoint(supportPolygon, vertex);
         finalListOfSupportPoints.add(correspondingSupportPoint);
      }
      return finalListOfSupportPoints;
   }

   private Point2D getPointInPolygonNearestPoint(ConvexPolygon2d polygon, Point2DReadOnly point2d)
   {
      LineSegment2d closestEdge = new LineSegment2d();
      polygon.getClosestEdge(closestEdge, point2d);
      return closestEdge.getClosestPointOnLineSegmentCopy(point2d);
   }
}
