package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Vector3d;

public class NotchFilteredYoFrameVector extends YoFrameVector implements ProcessingYoVariable
{
   private final NotchFilteredYoVariable x, y, z;

   private NotchFilteredYoFrameVector(NotchFilteredYoVariable x, NotchFilteredYoVariable y, NotchFilteredYoVariable z, ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static NotchFilteredYoFrameVector createNotchFilteredYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double dt,
         double naturalFrequencyHz, double dampingRatio, ReferenceFrame referenceFrame)
   {
      NotchFilteredYoVariableParameters parameters = new NotchFilteredYoVariableParameters(namePrefix + nameSuffix, registry, naturalFrequencyHz, dampingRatio);
      return createNotchFilteredYoFrameVector(namePrefix, nameSuffix, registry, dt, parameters, referenceFrame);
   }

   public static NotchFilteredYoFrameVector createNotchFilteredYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double dt,
         NotchFilteredYoVariableParameters parameters, ReferenceFrame referenceFrame)
   {
      NotchFilteredYoVariable x, y, z;
      x = new NotchFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, dt, parameters);
      y = new NotchFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, dt, parameters);
      z = new NotchFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, dt, parameters);
      return new NotchFilteredYoFrameVector(x, y, z, referenceFrame);
   }

   public static NotchFilteredYoFrameVector createNotchFilteredYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double dt,
         double naturalFrequencyHz, double dampingRatio, YoFrameVector unfilteredVector)
   {
      NotchFilteredYoVariableParameters parameters = new NotchFilteredYoVariableParameters(namePrefix + nameSuffix, registry, naturalFrequencyHz, dampingRatio);
      return createNotchFilteredYoFrameVector(namePrefix, nameSuffix, registry, dt, parameters, unfilteredVector);
   }

   public static NotchFilteredYoFrameVector createNotchFilteredYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double dt,
         NotchFilteredYoVariableParameters parameters, YoFrameVector unfilteredVector)
   {
      NotchFilteredYoVariable x, y, z;
      x = new NotchFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, dt, parameters, unfilteredVector.getYoX());
      y = new NotchFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, dt, parameters, unfilteredVector.getYoY());
      z = new NotchFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, dt, parameters, unfilteredVector.getYoZ());
      return new NotchFilteredYoFrameVector(x, y, z, unfilteredVector.getReferenceFrame());
   }

   public void update()
   {
      x.update();
      y.update();
      z.update();
   }

   public void update(double xUnfiltered, double yUnfiltered, double zUnfiltered)
   {
      x.update(xUnfiltered);
      y.update(yUnfiltered);
      z.update(zUnfiltered);
   }

   public void update(Vector3d vectorUnfiltered)
   {
      x.update(vectorUnfiltered.x);
      y.update(vectorUnfiltered.y);
      z.update(vectorUnfiltered.z);
   }

   public void update(FrameVector vectorUnfiltered)
   {
      checkReferenceFrameMatch(vectorUnfiltered);
      x.update(vectorUnfiltered.getX());
      y.update(vectorUnfiltered.getY());
      z.update(vectorUnfiltered.getZ());
   }

   public void reset()
   {
      x.reset();
      y.reset();
      z.reset();
   }
}
