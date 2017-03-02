package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class PrivilegedConfigurationCommand implements InverseKinematicsCommand<PrivilegedConfigurationCommand>, InverseDynamicsCommand<PrivilegedConfigurationCommand>
{
   private final int initialCapacity = 40;
   private final List<String> jointNames = new ArrayList<>(initialCapacity);
   private final List<OneDoFJoint> joints = new ArrayList<>(initialCapacity);
   private final TDoubleArrayList privilegedOneDoFJointConfigurations;
   private final TLongObjectHashMap<PrivilegedConfigurationOption> privilegedOneDoFJointConfigurationOptions;

   private boolean enable = false;

   public enum PrivilegedConfigurationOption
   {
      AT_CURRENT, AT_MID_RANGE, AT_ZERO
   }

   private PrivilegedConfigurationOption defaultOption;
   private double weight = Double.NaN;
   private double configurationGain = Double.NaN;
   private double velocityGain = Double.NaN;
   private double maxVelocity = Double.NaN;
   private double maxAcceleration = Double.NaN;

   private double defaultWeight = Double.NaN;
   private double defaultConfigurationGain = Double.NaN;
   private double defaultVelocityGain = Double.NaN;
   private double defaultMaxVelocity = Double.NaN;
   private double defaultMaxAcceleration = Double.NaN;

   public PrivilegedConfigurationCommand()
   {
      privilegedOneDoFJointConfigurations = new TDoubleArrayList(initialCapacity);
      privilegedOneDoFJointConfigurationOptions = new TLongObjectHashMap<PrivilegedConfigurationOption>(initialCapacity);
      
      clear();
   }

   public void clear()
   {
      enable = false;
      defaultOption = null;
      weight = Double.NaN;
      configurationGain = Double.NaN;
      velocityGain = Double.NaN;
      maxVelocity = Double.NaN;
      maxAcceleration = Double.NaN;
      jointNames.clear();
      joints.clear();
      privilegedOneDoFJointConfigurations.reset();
      privilegedOneDoFJointConfigurationOptions.clear();
   }

   public void disable()
   {
      enable = false;
   }

   public void enable()
   {
      enable = true;
   }

   public void setDefaultWeight(double defaultWeight)
   {
      this.defaultWeight = defaultWeight;
   }

   public void setDefaultConfigurationGain(double defaultConfigurationGain)
   {
      this.defaultConfigurationGain = defaultConfigurationGain;
   }

   public void setDefaultVelocityGain(double defaultVelocityGain)
   {
      this.defaultVelocityGain = defaultVelocityGain;
   }

   public void setDefaultMaxVelocity(double defaultMaxVelocity)
   {
      this.defaultMaxVelocity = defaultMaxVelocity;
   }

   public void setDefaultMaxAcceleration(double defaultMaxAcceleration)
   {
      this.defaultMaxAcceleration = defaultMaxAcceleration;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void setConfigurationGain(double configurationGain)
   {
      this.configurationGain = configurationGain;
   }

   public void setVelocityGain(double velocityGain)
   {
      this.velocityGain = velocityGain;
   }

   public void setMaxVelocity(double maxVelocity)
   {
      this.maxVelocity = maxVelocity;
   }

   public void setMaxAcceleration(double maxAcceleration)
   {
      this.maxAcceleration = maxAcceleration;
   }

   public void setPrivilegedConfigurationOption(PrivilegedConfigurationOption option)
   {
      enable();
      this.defaultOption = option;
   }

   public void addJoint(OneDoFJoint joint, double privilegedConfiguration)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add(privilegedConfiguration);
      privilegedOneDoFJointConfigurationOptions.put(joint.nameBasedHashCode(), null);
   }

   public void addJoint(OneDoFJoint joint, PrivilegedConfigurationOption privilegedConfiguration)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add(Double.NaN);
      privilegedOneDoFJointConfigurationOptions.put(joint.nameBasedHashCode(), privilegedConfiguration);
   }

   @Override
   public void set(PrivilegedConfigurationCommand other)
   {
      clear();
      enable = other.enable;
      defaultOption = other.defaultOption;

      weight = other.weight;
      configurationGain = other.configurationGain;
      velocityGain = other.velocityGain;
      maxVelocity = other.maxVelocity;
      maxAcceleration = other.maxAcceleration;

      defaultWeight = other.defaultWeight;
      defaultConfigurationGain = other.defaultConfigurationGain;
      defaultVelocityGain = other.defaultVelocityGain;
      defaultMaxVelocity = other.defaultMaxVelocity;
      defaultMaxAcceleration = other.defaultMaxAcceleration;

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         OneDoFJoint joint = other.joints.get(i);
         joints.add(joint);
         jointNames.add(other.jointNames.get(i));
         privilegedOneDoFJointConfigurations.add(other.privilegedOneDoFJointConfigurations.get(i));
         privilegedOneDoFJointConfigurationOptions.put(joint.nameBasedHashCode(), other.privilegedOneDoFJointConfigurationOptions.get(joint.nameBasedHashCode()));
      }
   }

   public boolean isEnabled()
   {
      return enable;
   }

   public boolean hasWeight()
   {
      return !Double.isNaN(weight);
   }

   public double getWeight()
   {
      return weight;
   }

   public boolean hasConfigurationGain()
   {
      return !Double.isNaN(configurationGain);
   }

   public double getConfigurationGain()
   {
      return configurationGain;
   }

   public boolean hasVelocityGain()
   {
      return !Double.isNaN(velocityGain);
   }

   public double getVelocityGain()
   {
      return velocityGain;
   }

   public boolean hasMaxVelocity()
   {
      return !Double.isNaN(maxVelocity);
   }

   public double getMaxVelocity()
   {
      return maxVelocity;
   }

   public boolean hasMaxAcceleration()
   {
      return !Double.isNaN(maxAcceleration);
   }

   public double getMaxAcceleration()
   {
      return maxAcceleration;
   }

   public boolean hasNewDefaultWeight()
   {
      return !Double.isNaN(defaultWeight);
   }

   public double getDefaultWeight()
   {
      return defaultWeight;
   }

   public boolean hasNewDefaultConfigurationGain()
   {
      return !Double.isNaN(defaultConfigurationGain);
   }

   public double getDefaultConfigurationGain()
   {
      return defaultConfigurationGain;
   }

   public boolean hasNewDefaultVelocityGain()
   {
      return !Double.isNaN(defaultVelocityGain);
   }

   public double getDefaultVelocityGain()
   {
      return defaultVelocityGain;
   }

   public boolean hasNewDefaultMaxVelocity()
   {
      return !Double.isNaN(defaultMaxVelocity);
   }

   public double getDefaultMaxVelocity()
   {
      return defaultMaxVelocity;
   }

   public boolean hasNewDefaultMaxAcceleration()
   {
      return !Double.isNaN(defaultMaxAcceleration);
   }

   public double getDefaultMaxAcceleration()
   {
      return defaultMaxAcceleration;
   }


   public boolean hasNewPrivilegedConfigurationDefaultOption()
   {
      return defaultOption != null;
   }

   public PrivilegedConfigurationOption getPrivilegedConfigurationDefaultOption()
   {
      return defaultOption;
   }

   public boolean hasNewPrivilegedConfiguration(int jointIndex)
   {
      return !Double.isNaN(privilegedOneDoFJointConfigurations.get(jointIndex));
   }

   public double getPrivilegedConfiguration(int jointIndex)
   {
      return privilegedOneDoFJointConfigurations.get(jointIndex);
   }

   public boolean hasNewPrivilegedConfigurationOption(int jointIndex)
   {
      return getPrivilegedConfigurationOption(jointIndex) != null;
   }

   public PrivilegedConfigurationOption getPrivilegedConfigurationOption(int jointIndex)
   {
      return privilegedOneDoFJointConfigurationOptions.get(joints.get(jointIndex).nameBasedHashCode());
   }

   public int getNumberOfJoints()
   {
      return joints.size();
   }

   public OneDoFJoint getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.PRIVILEGED_CONFIGURATION;
   }
}
