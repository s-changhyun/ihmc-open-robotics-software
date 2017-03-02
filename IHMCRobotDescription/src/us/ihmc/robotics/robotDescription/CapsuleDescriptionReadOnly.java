package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.LineSegment3d;

public class CapsuleDescriptionReadOnly implements ConvexShapeDescription
{
   private final double radius;
   private final LineSegment3d capToCapLineSegment = new LineSegment3d();

   public CapsuleDescriptionReadOnly(double radius, double height, RigidBodyTransform transformToCenter)
   {
      this(radius, height, Axis.Z, transformToCenter);
   }

   public CapsuleDescriptionReadOnly(double radius, LineSegment3d capToCapLineSegment, RigidBodyTransform transformToCenter)
   {
      this.radius = radius;
      this.capToCapLineSegment.set(capToCapLineSegment);
      capToCapLineSegment.applyTransform(transformToCenter);
   }

   public CapsuleDescriptionReadOnly(double radius, double height, Axis longAxis, RigidBodyTransform transformToCenter)
   {
      if (height < 2.0 * radius)
         throw new RuntimeException("Capsule height must be at least 2.0 * radius!");
      this.radius = radius;

      switch (longAxis)
      {
      case X:
      {
         this.capToCapLineSegment.set(-height / 2.0 + radius, 0.0, 0.0, 0.0, 0.0, height / 2.0 - radius);
         break;
      }
      case Y:
      {
         this.capToCapLineSegment.set(0.0, -height / 2.0 + radius, 0.0, 0.0, height / 2.0 - radius, 0.0);
         break;
      }
      case Z:
      {
         this.capToCapLineSegment.set(0.0, 0.0, -height / 2.0 + radius, 0.0, 0.0, height / 2.0 - radius);
         break;
      }
      }

      capToCapLineSegment.applyTransform(transformToCenter);
   }

   public double getRadius()
   {
      return radius;
   }

   public void getCapToCapLineSegment(LineSegment3d lineSegmentToPack)
   {
      lineSegmentToPack.set(capToCapLineSegment);
   }

}
