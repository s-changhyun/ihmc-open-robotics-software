package us.ihmc.simulationconstructionset.robotcommprotocol;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.variable.YoVariable;


public interface ReceivedDataListener
{
   public void receivedData(ArrayList<YoVariable<?>> sendVariables);
}
