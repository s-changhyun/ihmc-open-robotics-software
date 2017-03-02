package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class CubicTimeMatrixTest
{
   private static final double epsilon = 0.00005;
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      CubicTimeMatrix cubicTimeMatrix = new CubicTimeMatrix();

      Assert.assertEquals("", 4, cubicTimeMatrix.numCols);
      Assert.assertEquals("", 1, cubicTimeMatrix.numRows);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTimeSetting()
   {
      CubicTimeMatrix cubicTimeMatrix = new CubicTimeMatrix();

      Random random = new Random();

      int iters = 100;

      for (int i = 0; i <  iters; i++)
      {
         double time = 10.0 * random.nextDouble();

         cubicTimeMatrix.setCurrentTime(time);

         Assert.assertEquals("", Math.pow(time, 3.0), cubicTimeMatrix.get(0, 0), epsilon);
         Assert.assertEquals("", Math.pow(time, 2.0), cubicTimeMatrix.get(0, 1), epsilon);
         Assert.assertEquals("", time, cubicTimeMatrix.get(0, 2), epsilon);
         Assert.assertEquals("", 1.0, cubicTimeMatrix.get(0, 3), epsilon);
      }
   }
}
