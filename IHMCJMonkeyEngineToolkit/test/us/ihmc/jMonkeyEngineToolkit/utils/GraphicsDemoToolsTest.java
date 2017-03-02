package us.ihmc.jMonkeyEngineToolkit.utils;

import static org.junit.Assert.assertNotNull;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;

@ContinuousIntegrationPlan(categories={IntegrationCategory.UI})
public class GraphicsDemoToolsTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.6)
	@Test(timeout = 30000)
   public void testCreatePointCloud()
   {
      List<Point3D> worldPoints = new ArrayList<Point3D>();
      
      for (int i = 0; i < 1000; i++)
      {
         worldPoints.add(new Point3D(1.0, 1.0, 1.0));
      }
      
      Graphics3DNode pointCloudNode = GraphicsDemoTools.createPointCloud("PointCloud", worldPoints, 0.001, YoAppearance.Green());
      
      assertNotNull("Point cloud node is null. ", pointCloudNode);
   }
}
