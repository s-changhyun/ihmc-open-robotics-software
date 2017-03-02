package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;

public interface SensorReader extends AuxiliaryRobotDataProvider
{
   public abstract void read();

   public abstract SensorOutputMapReadOnly getSensorOutputMapReadOnly();

   public abstract SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly();
   

}