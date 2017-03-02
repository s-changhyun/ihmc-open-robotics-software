package us.ihmc.simulationconstructionset.util.environments;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.Sphere3d;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;


public class ContactableSphereRobot extends ContactableRobot
{
   private static final double DEFAULT_RADIUS = 0.5;
   private static final double DEFAULT_MASS = 10.0;

   private final FloatingJoint floatingJoint;
   private final Sphere3d originalSphere3d, currentSphere3d;

   private Link sphereLink;

   public ContactableSphereRobot()
   {
      this("ContactableSphereRobot");
   }

   public ContactableSphereRobot(String name)
   {
      this(name, DEFAULT_RADIUS, DEFAULT_MASS);
   }

   public ContactableSphereRobot(String name, double radius, double mass)
   {
      super(name);

      floatingJoint = new FloatingJoint("base", new Vector3D(0.0, 0.0, 0.0), this);

      sphereLink = ball(radius, mass);
      floatingJoint.setLink(sphereLink);
      this.addRootJoint(floatingJoint);

      originalSphere3d = new Sphere3d(radius);
      currentSphere3d = new Sphere3d(radius);
   }

   private Link ball(double radius, double mass)
   {
      Link ret = new Link("ball");

      ret.setMass(mass);

      ret.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, radius, radius, radius));

      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(radius, YoAppearance.EarthTexture());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   @Override
   public synchronized boolean isPointOnOrInside(Point3D pointInWorldToCheck)
   {
      return currentSphere3d.isInsideOrOnSurface(pointInWorldToCheck);
   }

   @Override
   public boolean isClose(Point3D pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public synchronized void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck)
   {
      currentSphere3d.checkIfInside(pointInWorldToCheck, intersectionToPack, normalToPack);
   }

   @Override
   public void setMass(double mass)
   {
      sphereLink.setMass(mass);
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      sphereLink.setMomentOfInertia(Ixx, Iyy, Izz);
   }

   @Override
   public FloatingJoint getFloatingJoint()
   {
      return floatingJoint;
   }


   @Override
   public void update()
   {
      super.update();
      updateCurrentSphere3d();
   }

   private final RigidBodyTransform temporaryTransform3D = new RigidBodyTransform();

   private synchronized void updateCurrentSphere3d()
   {
      floatingJoint.getTransformToWorld(temporaryTransform3D);
      currentSphere3d.set(originalSphere3d);
      currentSphere3d.applyTransform(temporaryTransform3D);
   }
}
