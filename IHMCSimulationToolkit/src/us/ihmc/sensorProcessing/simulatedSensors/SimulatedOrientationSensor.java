package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class SimulatedOrientationSensor extends SimulatedSensor<RotationMatrix>
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame frameUsedForPerfectOrientation;

   private final Quaternion tempQuaternionOne = new Quaternion();
   private final Quaternion tempQuaternionTwo = new Quaternion();

   private final AxisAngle tempAxisAngle = new AxisAngle();
   
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   private final YoFrameQuaternion yoFrameQuaternionPerfect, yoFrameQuaternionNoisy;
   private final DoubleYoVariable rotationAngleNoise;
   
   private final ControlFlowOutputPort<RotationMatrix> orientationOutputPort = createOutputPort("orientationOutputPort");

   public SimulatedOrientationSensor(String name, ReferenceFrame frameUsedForPerfectOrientation, YoVariableRegistry registry)
   {
      this.frameUsedForPerfectOrientation = frameUsedForPerfectOrientation;
      this.yoFrameQuaternionPerfect = new YoFrameQuaternion(name + "Perfect", ReferenceFrame.getWorldFrame(), registry);
      this.yoFrameQuaternionNoisy = new YoFrameQuaternion(name + "Noisy", ReferenceFrame.getWorldFrame(), registry);
            
      rotationAngleNoise = new DoubleYoVariable(name + "Noise", registry);
   }

   public void startComputation()
   {
      frameUsedForPerfectOrientation.getTransformToDesiredFrame(worldFrame).getRotation(rotationMatrix);
      yoFrameQuaternionPerfect.set(rotationMatrix);

      corrupt(rotationMatrix);
      orientationOutputPort.setData(rotationMatrix);
      yoFrameQuaternionNoisy.set(rotationMatrix);
      
      yoFrameQuaternionPerfect.get(tempQuaternionOne);
      yoFrameQuaternionNoisy.get(tempQuaternionTwo);
      
      tempQuaternionTwo.inverse();
      tempQuaternionOne.multiply(tempQuaternionTwo);
      
      tempAxisAngle.set(tempQuaternionOne);
      
      double noiseAngle = tempAxisAngle.getAngle();
      if (noiseAngle > Math.PI) noiseAngle = noiseAngle - 2.0 * Math.PI;
      if (noiseAngle < -Math.PI) noiseAngle = noiseAngle + 2.0 * Math.PI;
      
      rotationAngleNoise.set(Math.abs(noiseAngle));
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<RotationMatrix> getOrientationOutputPort()
   {
      return orientationOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}
