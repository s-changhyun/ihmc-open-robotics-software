package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.robotics.controllers.GainCalculator;
import us.ihmc.robotics.controllers.MatrixUpdater;
import us.ihmc.robotics.controllers.PositionPIDGainsInterface;
import us.ihmc.robotics.controllers.TangentialDampingGains;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.controllers.YoTangentialDampingGains;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class YoFootPositionGains implements YoPositionPIDGainsInterface
{
   private final DoubleYoVariable proportionalXYGain, proportionalZGain;
   private final DoubleYoVariable derivativeXYGain, derivativeZGain;
   private final DoubleYoVariable dampingRatio;

   private final DoubleYoVariable maximumFeedback;
   private final DoubleYoVariable maximumFeedbackRate;
   private final DoubleYoVariable maxDerivativeError;
   private final DoubleYoVariable maxProportionalError;

   private final YoTangentialDampingGains tangentialDampingGains;

   public YoFootPositionGains(String suffix, YoVariableRegistry registry)
   {
      proportionalXYGain = new DoubleYoVariable("kpXYLinear" + suffix, registry);
      proportionalZGain = new DoubleYoVariable("kpZLinear" + suffix, registry);
      derivativeXYGain = new DoubleYoVariable("kdXYLinear" + suffix, registry);
      derivativeZGain = new DoubleYoVariable("kdZLinear" + suffix, registry);
      dampingRatio = new DoubleYoVariable("zetaLinear" + suffix, registry);

      maximumFeedback = new DoubleYoVariable("maximumLinearFeedback" + suffix, registry);
      maximumFeedbackRate = new DoubleYoVariable("maximumLinearFeedbackRate" + suffix, registry);
      maxDerivativeError = new DoubleYoVariable("maximumLinearDerivativeError" + suffix, registry);
      maxProportionalError = new DoubleYoVariable("maximumLinearProportionalError" + suffix, registry);

      tangentialDampingGains = new YoTangentialDampingGains(suffix, registry);

      maximumFeedback.set(Double.POSITIVE_INFINITY);
      maximumFeedbackRate.set(Double.POSITIVE_INFINITY);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);
   }

   @Override
   public void reset()
   {
      proportionalXYGain.set(0.0);
      proportionalZGain.set(0.0);
      derivativeXYGain.set(0.0);
      derivativeZGain.set(0.0);
      dampingRatio.set(0.0);
      maximumFeedback.set(Double.POSITIVE_INFINITY);
      maximumFeedbackRate.set(Double.POSITIVE_INFINITY);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);
   }

   @Override
   public Matrix3DReadOnly createProportionalGainMatrix()
   {
      Matrix3D proportionalGainMatrix = new Matrix3D();

      proportionalXYGain.addVariableChangedListener(new MatrixUpdater(0, 0, proportionalGainMatrix));
      proportionalXYGain.addVariableChangedListener(new MatrixUpdater(1, 1, proportionalGainMatrix));
      proportionalZGain.addVariableChangedListener(new MatrixUpdater(2, 2, proportionalGainMatrix));

      proportionalXYGain.notifyVariableChangedListeners();
      proportionalZGain.notifyVariableChangedListeners();

      return proportionalGainMatrix;
   }

   @Override
   public Matrix3DReadOnly createDerivativeGainMatrix()
   {
      Matrix3D derivativeGainMatrix = new Matrix3D();

      derivativeXYGain.addVariableChangedListener(new MatrixUpdater(0, 0, derivativeGainMatrix));
      derivativeXYGain.addVariableChangedListener(new MatrixUpdater(1, 1, derivativeGainMatrix));
      derivativeZGain.addVariableChangedListener(new MatrixUpdater(2, 2, derivativeGainMatrix));

      derivativeXYGain.notifyVariableChangedListeners();
      derivativeZGain.notifyVariableChangedListeners();

      return derivativeGainMatrix;
   }

   @Override
   public Matrix3DReadOnly createIntegralGainMatrix()
   {
      return new Matrix3D();
   }

   public void createDerivativeGainUpdater(boolean updateNow)
   {
      VariableChangedListener kdXYUpdater = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            derivativeXYGain.set(GainCalculator.computeDerivativeGain(proportionalXYGain.getDoubleValue(), dampingRatio.getDoubleValue()));
         }
      };

      proportionalXYGain.addVariableChangedListener(kdXYUpdater);
      dampingRatio.addVariableChangedListener(kdXYUpdater);

      if (updateNow)
         kdXYUpdater.variableChanged(null);

      VariableChangedListener kdZUpdater = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            derivativeZGain.set(GainCalculator.computeDerivativeGain(proportionalZGain.getDoubleValue(), dampingRatio.getDoubleValue()));
         }
      };

      proportionalZGain.addVariableChangedListener(kdZUpdater);
      dampingRatio.addVariableChangedListener(kdZUpdater);

      if (updateNow)
         kdZUpdater.variableChanged(null);
   }

   @Override
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalXYGain.set(proportionalGainX);
      proportionalZGain.set(proportionalGainZ);
   }

   public void setProportionalGains(double proportionalGainXY, double proportionalGainZ)
   {
      proportionalXYGain.set(proportionalGainXY);
      proportionalZGain.set(proportionalGainZ);
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeXYGain.set(derivativeGainX);
      derivativeZGain.set(derivativeGainZ);
   }

   public void setDerivativeGains(double derivativeGainXY, double derivativeGainZ)
   {
      derivativeXYGain.set(derivativeGainXY);
      derivativeZGain.set(derivativeGainZ);
   }

   public void setDampingRatio(double dampingRatio)
   {
      this.dampingRatio.set(dampingRatio);
   }

   @Override
   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
   }

   @Override
   public void setProportionalGains(double[] proportionalGains)
   {
      setProportionalGains(proportionalGains[0], proportionalGains[1], proportionalGains[2]);
   }

   @Override
   public void setDerivativeGains(double[] derivativeGains)
   {
      setDerivativeGains(derivativeGains[0], derivativeGains[1], derivativeGains[2]);
   }

   @Override
   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
   }

   @Override
   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      maximumFeedback.set(maxFeedback);
      maximumFeedbackRate.set(maxFeedbackRate);
   }

   @Override
   public void setMaxDerivativeError(double maxDerivativeError)
   {
      this.maxDerivativeError.set(maxDerivativeError);
   }

   @Override
   public void setMaxProportionalError(double maxProportionalError)
   {
      this.maxProportionalError.set(maxProportionalError);
   }

   @Override
   public void setTangentialDampingGains(TangentialDampingGains tangentialDampingGains)
   {
      this.tangentialDampingGains.set(tangentialDampingGains);
   }

   @Override
   public void setTangentialDampingGains(double kdReductionRatio, double parallelDampingDeadband, double positionErrorForMinimumKd)
   {
      tangentialDampingGains.set(kdReductionRatio, parallelDampingDeadband, positionErrorForMinimumKd);
   }

   @Override
   public void set(PositionPIDGainsInterface gains)
   {
      setProportionalGains(gains.getProportionalGains());
      setDerivativeGains(gains.getDerivativeGains());
      setIntegralGains(gains.getIntegralGains(), gains.getMaximumIntegralError());
      setTangentialDampingGains(gains.getTangentialDampingGains());
      setMaxFeedbackAndFeedbackRate(gains.getMaximumFeedback(), gains.getMaximumFeedbackRate());
      setMaxDerivativeError(gains.getMaximumDerivativeError());
      setMaxProportionalError(gains.getMaximumProportionalError());
   }

   @Override
   public DoubleYoVariable getYoMaximumFeedback()
   {
      return maximumFeedback;
   }

   @Override
   public DoubleYoVariable getYoMaximumFeedbackRate()
   {
      return maximumFeedbackRate;
   }

   @Override
   public DoubleYoVariable getYoMaximumDerivativeError()
   {
      return maxDerivativeError;
   }

   @Override
   public DoubleYoVariable getYoMaximumProportionalError()
   {
      return maxProportionalError;
   }

   @Override
   public YoTangentialDampingGains getYoTangentialDampingGains()
   {
      return tangentialDampingGains;
   }

   @Override
   public TangentialDampingGains getTangentialDampingGains()
   {
      return tangentialDampingGains;
   }

   private double[] tempProportionalGains = new double[3];

   @Override
   public double[] getProportionalGains()
   {
      tempProportionalGains[0] = proportionalXYGain.getDoubleValue();
      tempProportionalGains[1] = proportionalXYGain.getDoubleValue();
      tempProportionalGains[2] = proportionalZGain.getDoubleValue();

      return tempProportionalGains;
   }

   private double[] tempDerivativeGains = new double[3];

   @Override
   public double[] getDerivativeGains()
   {
      tempDerivativeGains[0] = derivativeXYGain.getDoubleValue();
      tempDerivativeGains[1] = derivativeXYGain.getDoubleValue();
      tempDerivativeGains[2] = derivativeZGain.getDoubleValue();

      return tempDerivativeGains;
   }

   private double[] tempIntegralGains = new double[3];

   @Override
   public double[] getIntegralGains()
   {
      return tempIntegralGains;
   }

   @Override
   public double getMaximumIntegralError()
   {
      return 0.0;
   }

   @Override
   public double getMaximumFeedback()
   {
      return maximumFeedback.getDoubleValue();
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maximumFeedbackRate.getDoubleValue();
   }

   @Override
   public double getMaximumDerivativeError()
   {
      return maxDerivativeError.getDoubleValue();
   }

   @Override
   public double getMaximumProportionalError()
   {
      return maxProportionalError.getDoubleValue();
   }

}
