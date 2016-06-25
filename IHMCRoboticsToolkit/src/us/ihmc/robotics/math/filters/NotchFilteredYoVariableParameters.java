package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class NotchFilteredYoVariableParameters
{
   private final DoubleYoVariable naturalFrequency;
   private final DoubleYoVariable dampingRatio;

   public NotchFilteredYoVariableParameters(String name, YoVariableRegistry registry, double naturalFrequencyInHz, double dampingRatio)
   {
      this.naturalFrequency = new DoubleYoVariable(name + "NaturalFrequency", registry);
      this.naturalFrequency.set(naturalFrequencyInHz);
      this.dampingRatio = new DoubleYoVariable(name + "DampingRatio", registry);
      this.dampingRatio.set(dampingRatio);
   }

   public DoubleYoVariable getNaturalFrequency()
   {
      return naturalFrequency;
   }

   public DoubleYoVariable getDampingRatio()
   {
      return dampingRatio;
   }
}
