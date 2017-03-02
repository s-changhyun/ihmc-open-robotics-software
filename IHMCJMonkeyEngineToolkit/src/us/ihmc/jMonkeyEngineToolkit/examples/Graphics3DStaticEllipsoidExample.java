package us.ihmc.jMonkeyEngineToolkit.examples;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapterTools;
import us.ihmc.robotics.geometry.shapes.Ellipsoid3d;

public class Graphics3DStaticEllipsoidExample
{
   private final ArrayList<Ellipsoid3d> ellipsoids = new ArrayList<Ellipsoid3d>();
   
   public void createWorld(Graphics3DAdapter graphics3DAdapter, Random random, int numberOfEllipsoids)
   {
      Point3D minValues = new Point3D(-5.0, -5.0, 0.0);
      Point3D maxValues = new Point3D(5.0, 5.0, 2.0);

      double minRadius = 0.1;
      double maxRadius = 0.5;
      
      for (int i=0; i<numberOfEllipsoids; i++)
      {
        Vector3D center = generateRandomVector3d(random, minValues, maxValues);
        RigidBodyTransform transform = new RigidBodyTransform();
        transform.setTranslation(center);
        double xRadius = generateRandomDoubleBetween(random, minRadius, maxRadius);
        double yRadius = generateRandomDoubleBetween(random, minRadius, maxRadius);
        double zRadius = generateRandomDoubleBetween(random, minRadius, maxRadius);
         
         Ellipsoid3d definition = new Ellipsoid3d(xRadius, yRadius, zRadius);
         
         ellipsoids.add(definition);
         
         Graphics3DNode node = new Graphics3DNode("node_" + i, Graphics3DNodeType.JOINT);
         Graphics3DObject ellipsoidObject = new Graphics3DObject();
         ellipsoidObject.translate(center);
         ellipsoidObject.addEllipsoid(xRadius, yRadius, zRadius, YoAppearance.Red());
         node.setGraphicsObject(ellipsoidObject);
         
         graphics3DAdapter.addRootNode(node);
      }    
      
      Graphics3DAdapterTools.createNewWindow(graphics3DAdapter, "Graphics3DStaticEllipsoidExample", 800, 600);
   }
   
   public boolean isPointNearSurfaceOfAnEllipsoid(Point3D point, double epsilon)
   {
      for (Ellipsoid3d ellipsoid : ellipsoids)
      {
         if (ellipsoid.isInsideOrOnSurface(point, epsilon)) return true;
      }

      return false;
   }
   
   public boolean isPointInsideAnEllipsoid(Point3D point, double epsilon)
   {
      for (Ellipsoid3d ellipsoid : ellipsoids)
      {
         if (ellipsoid.isInsideOrOnSurface(point, epsilon)) return true;
      }

      return false;
   }
   
   private double generateRandomDoubleBetween(Random random, double minValue, double maxValue)
   {
      return minValue + random.nextDouble() * (maxValue - minValue);
   }
   
   private Vector3D generateRandomVector3d(Random random, Point3D minValues, Point3D maxValues)
   {
      double x = generateRandomDoubleBetween(random, minValues.getX(), maxValues.getX());
      double y = generateRandomDoubleBetween(random, minValues.getY(), maxValues.getY());
      double z = generateRandomDoubleBetween(random, minValues.getZ(), maxValues.getZ());
      
      return new Vector3D(x, y, z);
   }
   
}
