package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/**
 * A discrete-time second order notch filter using the bilinear transform
 *
 * Y(s)             s^2 + omega^2
 * ---- =  ----------------------------------
 * X(s)    s^2 + 2 * xi * omega * s + omega^2
 *
 * omega = 2 * PI * naturalFrequency
 * xi = dampingRatio
 */
public class NotchFilteredYoVariable extends DoubleYoVariable implements ProcessingYoVariable
{
   private final double dt;
   private final NotchFilteredYoVariableParameters parameters;
   protected final BooleanYoVariable hasBeenCalled;
   private final DoubleYoVariable inputVariable;
   private final DoubleYoVariable[] input;
   private final DoubleYoVariable[] output;
   private final double a[];
   private final double b[];

   public NotchFilteredYoVariable(String name, YoVariableRegistry registry, double dt, double naturalFrequencyHz, double dampingRatio)
   {
      this(name, registry, dt, new NotchFilteredYoVariableParameters(name, registry, naturalFrequencyHz, dampingRatio), null);
   }

   public NotchFilteredYoVariable(String name, YoVariableRegistry registry, double dt, NotchFilteredYoVariableParameters parameters)
   {
      this(name, registry, dt, parameters, null);
   }

   public NotchFilteredYoVariable(String name, YoVariableRegistry registry, double dt, double naturalFrequencyHz, double dampingRatio,
         DoubleYoVariable inputVariable)
   {
      this(name, registry, dt, new NotchFilteredYoVariableParameters(name, registry, naturalFrequencyHz, dampingRatio), inputVariable);
   }

   public NotchFilteredYoVariable(String name, YoVariableRegistry registry, double dt, NotchFilteredYoVariableParameters parameters,
         DoubleYoVariable inputVariable)
   {
      super(name, registry);
      this.dt = dt;
      this.parameters = parameters;
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);
      this.inputVariable = inputVariable;
      this.input = new DoubleYoVariable[3];
      this.output = new DoubleYoVariable[3];
      this.a = new double[3];
      this.b = new double[3];
      for (int i = 0; i < 3; i++)
      {
         this.input[i] = new DoubleYoVariable(name + "input" + i, registry);
         this.output[i] = new DoubleYoVariable(name + "output" + i, registry);
      }
      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
      computeCoefficients();
   }

   public void update()
   {
      if (inputVariable == null)
      {
         throw new NullPointerException(
               "YoNotchFilteredVariable must be constructed with a non null " + "position variable to call update(), otherwise use update(double)");
      }

      update(inputVariable.getDoubleValue());
   }

   public void update(double currentInputValue)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(currentInputValue);
         for (int i = 0; i < 3; i++)
         {
            input[i].set(currentInputValue);
            output[i].set(currentInputValue);
         }
         return;
      }

      for (int i = 2; i > 0; i--)
      {
         input[i].set(input[i - 1].getDoubleValue());
         output[i].set(output[i - 1].getDoubleValue());
      }
      input[0].set(currentInputValue);

      double currentOutputValue = 0.0;
      currentOutputValue += b[2] * input[2].getDoubleValue();
      currentOutputValue += b[1] * input[1].getDoubleValue();
      currentOutputValue += b[0] * input[0].getDoubleValue();
      currentOutputValue -= a[2] * output[2].getDoubleValue();
      currentOutputValue -= a[1] * output[1].getDoubleValue();
      currentOutputValue /= a[0];
      output[0].set(currentOutputValue);

      set(currentOutputValue);
   }

   public void setNaturalFrequency(double naturalFrequencyHz)
   {
      parameters.getNaturalFrequency().set(Math.min(Math.max(naturalFrequencyHz, 0), 1.0 / (2.0 * dt)));
      computeCoefficients();
   }

   public void setDampingRatio(double dampingRatio)
   {
      parameters.getDampingRatio().set(Math.max(dampingRatio, 0));
      computeCoefficients();
   }

   public boolean getHasBeenCalled()
   {
      return hasBeenCalled.getBooleanValue();
   }

   private void computeCoefficients()
   {
      double omega = 2 * Math.PI * parameters.getNaturalFrequency().getDoubleValue();
      double xi = parameters.getDampingRatio().getDoubleValue();

      b[0] = 4 / (dt * dt) + omega * omega;
      b[1] = 2 * omega * omega - 8 / (dt * dt);
      b[2] = 4 / (dt * dt) + omega * omega;
      a[0] = 4 / (dt * dt) + 4 / dt * xi * omega + omega * omega;
      a[1] = 2 * omega * omega - 8 / (dt * dt);
      a[2] = 4 / (dt * dt) - 4 / dt * xi * omega + omega * omega;
   }
}
