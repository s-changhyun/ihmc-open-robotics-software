package us.ihmc.robotics.geometry;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Represents a finite-length 3D line segment defined by its two 3D endpoints.
 * 
 * @author Sylvain Bertrand
 *
 */
public class LineSegment3d implements GeometryObject<LineSegment3d>
{
   private final Point3D firstEndpoint = new Point3D();
   private final Point3D secondEndpoint = new Point3D();

   /**
    * Default constructor that initializes both endpoints of this line segment to zero.
    */
   public LineSegment3d()
   {
   }

   /**
    * Initializes this line segment to have the given endpoints.
    * 
    * @param firstEndpoint the first endpoint of this line segment. Not modified.
    * @param secondEndpoint the second endpoint of this line segment. Not modified.
    */
   public LineSegment3d(Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      set(firstEndpoint, secondEndpoint);
   }

   /**
    * Initializes this line segment to have the given endpoints.
    * 
    * @param firstEndpointX x-coordinate of the first endpoint of this line segment.
    * @param firstEndpointY y-coordinate of the first endpoint of this line segment.
    * @param firstEndpointZ z-coordinate of the first endpoint of this line segment.
    * @param secondEndpointX x-coordinate of the second endpoint of this line segment.
    * @param secondEndpointY y-coordinate of the second endpoint of this line segment.
    * @param secondEndpointZ z-coordinate of the second endpoint of this line segment.
    */
   public LineSegment3d(double firstEndpointX, double firstEndpointY, double firstEndpointZ, double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      set(firstEndpointX, firstEndpointY, firstEndpointZ, secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   /**
    * Changes the first endpoint of this line segment.
    * 
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param firstEndpointZ z-coordinate of the new first endpoint.
    */
   public void setFirstEndpoint(double firstEndpointX, double firstEndpointY, double firstEndpointZ)
   {
      firstEndpoint.set(firstEndpointX, firstEndpointY, firstEndpointZ);
   }

   /**
    * Changes the first endpoint of this line segment.
    * 
    * @param firstEndpoint new endpoint of this line segment. Not modified
    */
   public void setFirstEndpoint(Point3DReadOnly firstEndpoint)
   {
      this.firstEndpoint.set(firstEndpoint);
   }

   /**
    * Changes the second endpoint of this line segment.
    * 
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    * @param secondEndpointZ z-coordinate of the new second endpoint.
    */
   public void setSecondEndpoint(double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      secondEndpoint.set(secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   /**
    * Changes the second endpoint of this line segment.
    * 
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   public void setSecondEndpoint(Point3DReadOnly secondEndpoint)
   {
      this.secondEndpoint.set(secondEndpoint);
   }

   /**
    * Redefines this line segments with new endpoints.
    * 
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param firstEndpointZ z-coordinate of the new first endpoint.
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    * @param secondEndpointZ z-coordinate of the new second endpoint.
    */
   public void set(double firstEndpointX, double firstEndpointY, double firstEndpointZ, double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      setFirstEndpoint(firstEndpointX, firstEndpointY, firstEndpointZ);
      setSecondEndpoint(secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   /**
    * Redefines this line segment with new endpoints.
    * 
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   public void set(Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going
    * from the first to the second endpoint.
    * 
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not modified.
    */
   public void set(Point3DReadOnly firstEndpoint, Vector3DReadOnly fromFirstToSecondEndpoint)
   {
      this.firstEndpoint.set(firstEndpoint);
      this.secondEndpoint.add(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Sets this line segment to be same as the given line segment.
    * 
    * @param other the other line segment to copy. Not modified.
    */
   @Override
   public void set(LineSegment3d other)
   {
      set(other.firstEndpoint, other.secondEndpoint);
   }

   /**
    * Sets both endpoints of this line segment to zero.
    */
   @Override
   public void setToZero()
   {
      firstEndpoint.setToZero();
      secondEndpoint.setToZero();
   }

   /**
    * Sets both endpoints of this line segment to {@link Double#NaN}.
    * After calling this method, this line segment becomes invalid.
    * A new pair of valid endpoints will have to be set so this line segment is again usable.
    */
   @Override
   public void setToNaN()
   {
      firstEndpoint.setToNaN();
      secondEndpoint.setToNaN();
   }

   /**
    * Tests if this line segment contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #firstEndpoint} and/or {@link #secondEndpoint} contains {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      return firstEndpoint.containsNaN() || secondEndpoint.containsNaN();
   }

   /**
    * Test if the first endpoint of this line segment contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #firstEndpoint} contains {@link Double#NaN}, {@code false} otherwise.
    */
   public boolean firstEndpointContainsNaN()
   {
      return firstEndpoint.containsNaN();
   }

   /**
    * Test if the second endpoint of this line segment contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #secondEndpoint} contains {@link Double#NaN}, {@code false} otherwise.
    */
   public boolean secondEndpointContainsNaN()
   {
      return secondEndpoint.containsNaN();
   }

   /**
    * Transforms this line segment using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(firstEndpoint);
      transform.transform(secondEndpoint);
   }

   /**
    * Computes the length of this line segment.
    * 
    * @return the length of this line segment.
    */
   public double length()
   {
      return firstEndpoint.distance(secondEndpoint);
   }

   /**
    * Returns the square of the minimum distance between a point and this given line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if {@code this.length() < Epsilons.ONE_TRILLIONTH}, this method returns the distance between {@code firstEndpoint} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from this line segment. Not modified.
    * @return the minimum distance between the 3D point and this 3D line segment.
    */
   public double distanceSquared(Point3DReadOnly point)
   {
      return GeometryTools.distanceFromPointToLineSegmentSquared(point, firstEndpoint, secondEndpoint);
   }

   /**
    * Returns the minimum distance between a point and this given line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if {@code this.length() < Epsilons.ONE_TRILLIONTH}, this method returns the distance between {@code firstEndpoint} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from this line segment. Not modified.
    * @return the minimum distance between the 3D point and this 3D line segment.
    */
   public double distance(Point3DReadOnly point)
   {
      return GeometryTools.distanceFromPointToLineSegment(point, firstEndpoint, secondEndpoint);
   }

   /**
    * This methods computes the minimum distance between this line segment and the given one.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * 
    * @param otherLineSegment the other line segment to compute the distance from. Not modified.
    * @return the minimum distance between the two line segments.
    */
   public double distance(LineSegment3d otherLineSegment)
   {
      return GeometryTools.distanceBetweenTwoLineSegments(firstEndpoint, secondEndpoint, otherLineSegment.firstEndpoint, otherLineSegment.secondEndpoint);
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of this line segment is too small,
    *     i.e. {@code this.length() < Epsilons.ONE_TRILLIONTH},
    *      this method returns {@code firstEndpoint}.
    *    <li> the projection can not be outside the line segment.
    *     When the projection on the corresponding line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the line segment or {@code null} if the method failed.
    */
   public Point3D orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(pointToProject, firstEndpoint, secondEndpoint);
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of this line segment is too small,
    *     i.e. {@code this.length() < Epsilons.ONE_TRILLIONTH},
    *      this method returns {@code firstEndpoint}.
    *    <li> the projection can not be outside the line segment.
    *     When the projection on the corresponding line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    * 
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is stored. Modified.
    * @return whether the method succeeded or not.
    */
   public boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(pointToProject, firstEndpoint, secondEndpoint, projectionToPack);
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment:
    * <br> {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @return the computed point.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   public Point3D pointBetweenEndPointsGivenPercentageCopy(double percentage)
   {
      Point3D point = new Point3D();
      pointBetweenEndPointsGivenPercentage(percentage, point);
      return point;
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment:
    * <br> {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * 
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @param pointToPack where the result is stored. Modified.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   public void pointBetweenEndPointsGivenPercentage(double percentage, Point3DBasics pointToPack)
   {
      if (percentage < 0.0 || percentage > 1.0)
         throw new RuntimeException("Percentage must be between 0.0 and 1.0. Was: " + percentage);

      pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage);
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on:
    * <br> {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param percentage the percentage along this line segment of the point.
    * @return the computed point.
    */
   public Point3D pointOnLineGivenPercentageCopy(double percentage)
   {
      Point3D point = new Point3D();
      pointOnLineGivenPercentage(percentage, point);
      return point;
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on:
    * <br> {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * 
    * @param percentage the percentage along this line segment of the point.
    * @param pointToPack where the result is stored. Modified.
    */
   public void pointOnLineGivenPercentage(double percentage, Point3DBasics pointToPack)
   {
      pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage);
   }

   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    * 
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    */
   public void getMidpoint(Point3DBasics midpointToPack)
   {
      midpointToPack.interpolate(firstEndpoint, secondEndpoint, 0.5);
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    * 
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   public void getDirection(boolean normalize, Vector3DBasics directionToPack)
   {
      directionToPack.sub(secondEndpoint, firstEndpoint);
      if (normalize)
         directionToPack.normalize();
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param normalize whether the direction vector is to be normalized.
    * @return the direction of this line segment.
    */
   public Vector3D getDirectionCopy(boolean normalize)
   {
      Vector3D direction = new Vector3D();
      getDirection(normalize, direction);
      return direction;
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located
    * between the two endpoints or exactly on an endpoint.
    * 
    * @param point the query. Not modified.
    * @return {@code true} if the projection of the point is between the endpoints of this line segment, {@code false} otherwise.
    */
   public boolean isBetweenEndpoints(Point3DReadOnly point)
   {
      return isBetweenEndpoints(point, 0);
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located
    * between the two endpoints with a given conservative tolerance {@code epsilon}:
    * <ul>
    *    <li> if {@code epsilon > 0}, the point has to be between the endpoints and at a minimum
    *     distance of {@code epsilon * this.length()} from the closest endpoint.
    *    <li> if {@code epsilon < 0}, the point has to be between the endpoints or at a maximum
    *     distance of {@code -epsilon * this.length()} from the closest endpoint.
    *    <li> if {@code epsilon = 0}, the point has to be between the endpoints or equal to
    *     one of the endpoints.
    * </ul>
    * 
    * @param point the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the projection of the point is between the endpoints of this line segment, {@code false} otherwise.
    */
   public boolean isBetweenEndpoints(Point3DReadOnly point, double epsilon)
   {
      return isBetweenEndpoints(point.getX(), point.getY(), point.getZ(), epsilon);
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located
    * between the two endpoints with a given conservative tolerance {@code epsilon}:
    * <ul>
    *    <li> if {@code epsilon > 0}, the point has to be between the endpoints and at a minimum
    *     distance of {@code epsilon * this.length()} from the closest endpoint.
    *    <li> if {@code epsilon < 0}, the point has to be between the endpoints or at a maximum
    *     distance of {@code -epsilon * this.length()} from the closest endpoint.
    *    <li> if {@code epsilon = 0}, the point has to be between the endpoints or equal to
    *     one of the endpoints.
    * </ul>
    * 
    * @param x the x-coordinate of the query point.
    * @param y the y-coordinate of the query point.
    * @param z the z-coordinate of the query point.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the projection of the point is between the endpoints of this line segment, {@code false} otherwise.
    */
   public boolean isBetweenEndpoints(double x, double y, double z, double epsilon)
   {
      double alpha = percentageAlongLineSegment(x, y, z);

      if (alpha < epsilon)
         return false;
      if (alpha > 1.0 - epsilon)
         return false;

      return true;
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once projected onto this line segment.
    * The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given point is located at the middle of this line segment.
    * The coordinates of the projection of the point can be computed from the {@code percentage} as follows:
    * <code>
    * Point3D projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of the given line segment is too small, i.e. {@code this.length() < Epsilons.ONE_TRILLIONTH}, this method fails and returns {@code 0.0}.
    * </ul>
    * </p>
    * 
    * @param point the query point. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is located.
    */
   public double percentageAlongLineSegment(Point3DReadOnly point)
   {
      return percentageAlongLineSegment(point.getX(), point.getY(), point.getZ());
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once projected onto this line segment.
    * The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given point is located at the middle of this line segment.
    * The coordinates of the projection of the point can be computed from the {@code percentage} as follows:
    * <code>
    * Point3D projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of the given line segment is too small, i.e. {@code this.length() < Epsilons.ONE_TRILLIONTH}, this method fails and returns {@code 0.0}.
    * </ul>
    * </p>
    * 
    * @param x the x-coordinate of the query point.
    * @param y the y-coordinate of the query point.
    * @param z the z-coordinate of the query point.
    * @return the computed percentage along the line segment representing where the point projection is located.
    */
   public double percentageAlongLineSegment(double x, double y, double z)
   {
      return GeometryTools.getPercentageAlongLineSegment(x, y, z, firstEndpoint.getX(), firstEndpoint.getY(), firstEndpoint.getZ(), secondEndpoint.getX(),
                                                         secondEndpoint.getY(), secondEndpoint.getZ());
   }

   /**
    * @return the reference to the first endpoint of this line segment.
    */
   public Point3D getFirstEndpoint()
   {
      return firstEndpoint;
   }

   /**
    * @return the reference to the second endpoint of this line segment.
    */
   public Point3D getSecondEndpoint()
   {
      return secondEndpoint;
   }

   /**
    * Computes the line on which this line segment is lying.
    * The line's vector is the direction from the first to the second endpoint
    * of this line segment.
    * 
    * @param lineToPack the line on which this line segment is lying.
    */
   public void getLine(Line3d lineToPack)
   {
      lineToPack.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Computes the line on which this line segment is lying.
    * The line's vector is the direction from the first to the second endpoint
    * of this line segment.
    * 
    * @return the line on which this line segment is lying.
    */
   public Line3d getLineCopy()
   {
      return new Line3d(firstEndpoint, secondEndpoint);
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to {@code other} with the tolerance {@code epsilon}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(LineSegment3d other, double epsilon)
   {
      return firstEndpoint.epsilonEquals(other.firstEndpoint, epsilon) && secondEndpoint.epsilonEquals(other.secondEndpoint, epsilon);
   }

   /**
    * Returns a {@code String} with the two endpoints of this line segment.
    */
   @Override
   public String toString()
   {
      return firstEndpoint + "-" + secondEndpoint;
   }
}
