package us.ihmc.steppr.hardware.visualization;

import java.util.EnumMap;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.IndexChangedListener;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprDashboard;
import us.ihmc.steppr.hardware.StepprJoint;

public class StepprPDSliderboard extends SCSVisualizer implements IndexChangedListener
{

   private final YoVariableRegistry sliderBoardRegistry = new YoVariableRegistry("StepprPDSliderBoard");
   private final EnumYoVariable<StepprJoint> selectedJoint = new EnumYoVariable<>("selectedJoint", sliderBoardRegistry, StepprJoint.class);

   private final BooleanYoVariable selectedJoint_enabled = new BooleanYoVariable("selectedJoint_enabled", sliderBoardRegistry);
   private final DoubleYoVariable selectedJoint_q = new DoubleYoVariable("selectedJoint_q", sliderBoardRegistry);
   private final DoubleYoVariable selectedJoint_qd = new DoubleYoVariable("selectedJoint_qd", sliderBoardRegistry);
   private final DoubleYoVariable selectedJoint_tau = new DoubleYoVariable("selectedJoint_tau", sliderBoardRegistry);
   private final DoubleYoVariable selectedJoint_tau_d = new DoubleYoVariable("selectedJoint_tau_d", sliderBoardRegistry);

   private final DoubleYoVariable selectedJoint_q_d = new DoubleYoVariable("selectedJoint_q_d", sliderBoardRegistry);
   private final DoubleYoVariable selectedJoint_qd_d = new DoubleYoVariable("selectedJoint_qd_d", sliderBoardRegistry);
   private final DoubleYoVariable selectedJoint_kp = new DoubleYoVariable("selectedJoint_kp", sliderBoardRegistry);
   private final DoubleYoVariable selectedJoint_kd = new DoubleYoVariable("selectedJoint_kd", sliderBoardRegistry);
   private final DoubleYoVariable selectedJoint_tauFF = new DoubleYoVariable("selectedJoint_tauFF", sliderBoardRegistry);
   private final DoubleYoVariable selectedJoint_damping = new DoubleYoVariable("selectedJoint_damping", sliderBoardRegistry);

   private volatile boolean started = false;

   private final EnumMap<StepprJoint, JointVariables> allJointVariables = new EnumMap<>(StepprJoint.class);

   public StepprPDSliderboard(int bufferSize)
   {
      super(bufferSize);

   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      scs.getDataBuffer().attachIndexChangedListener(this);
      registry.addChild(sliderBoardRegistry);
      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

      for (StepprJoint joint : StepprJoint.values)
      {
         JointVariables variables = new JointVariables(joint, registry);

         OneDegreeOfFreedomJoint oneDoFJoint = ((FloatingRootJointRobot) robot).getOneDegreeOfFreedomJoint(joint.getSdfName());
         sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, StepprJoint.values.length);
         sliderBoardConfigurationManager.setSlider(1, variables.q_d, oneDoFJoint.getJointLowerLimit(), oneDoFJoint.getJointUpperLimit());
         sliderBoardConfigurationManager.setSlider(2, variables.qd_d, -1, 1);
         sliderBoardConfigurationManager.setSlider(3, variables.kp, 0, 10 * joint.getRatio() * joint.getRatio());
         sliderBoardConfigurationManager.setSlider(4, variables.kd, -.1 * joint.getRatio() * joint.getRatio(), .1 * joint.getRatio() * joint.getRatio());

         if (Double.isNaN(oneDoFJoint.getTorqueLimit()) || Double.isInfinite(oneDoFJoint.getTorqueLimit()))
         {
            sliderBoardConfigurationManager.setSlider(5, variables.tauFF, -50, 50);
         }
         else
         {
            sliderBoardConfigurationManager.setSlider(5, variables.tauFF, -oneDoFJoint.getTorqueLimit(), oneDoFJoint.getTorqueLimit());
         }
         sliderBoardConfigurationManager.setSlider(6, variables.damping, 0, .5 * joint.getRatio() * joint.getRatio());

         sliderBoardConfigurationManager.setButton(1, variables.enabled);
         sliderBoardConfigurationManager.saveConfiguration(joint.toString());
         allJointVariables.put(joint, variables);
      }

      selectedJoint.addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
         }
      });

      selectedJoint.set(StepprJoint.RIGHT_KNEE_Y);

      StepprDashboard.createDashboard(scs, registry);

      started = true;
   }

   private class JointVariables
   {
      private final BooleanYoVariable enabled;

      private final DoubleYoVariable q;
      private final DoubleYoVariable qd;
      private final DoubleYoVariable tau;
      private final DoubleYoVariable tau_d;

      private final DoubleYoVariable q_d;
      private final DoubleYoVariable qd_d;

      private final DoubleYoVariable kp;
      private final DoubleYoVariable kd;

      private final DoubleYoVariable tauFF;
      private final DoubleYoVariable damping;

      public JointVariables(final StepprJoint joint, final YoVariableHolder variableHolder)
      {
         final String prefix = joint.getSdfName();
         final String namespace = "StepprCommand." + prefix;
         enabled = new BooleanYoVariable(joint.getSdfName() + "_enabled", sliderBoardRegistry);
         enabled.addVariableChangedListener(new VariableChangedListener()
         {

            @Override
            public void variableChanged(YoVariable<?> v)
            {
               for (StepprActuator actuator : joint.getActuators())
               {
                  String namespace;
                  String variable;
                  switch (actuator)
                  {
                  case LEFT_ANKLE_RIGHT:
                     namespace = "StepprCommand.leftAnkleCommand.leftAnkleCommandRightActuator";
                     variable = "leftAnkleCommandRightActuatorEnabled";
                     break;
                  case LEFT_ANKLE_LEFT:
                     namespace = "StepprCommand.leftAnkleCommand.leftAnkleCommandLeftActuator";
                     variable = "leftAnkleCommandLeftActuatorEnabled";
                     break;
                  case RIGHT_ANKLE_RIGHT:
                     namespace = "StepprCommand.rightAnkleCommand.rightAnkleCommandRightActuator";
                     variable = "rightAnkleCommandRightActuatorEnabled";
                     break;
                  case RIGHT_ANKLE_LEFT:
                     namespace = "StepprCommand.rightAnkleCommand.rightAnkleCommandLeftActuator";
                     variable = "rightAnkleCommandLeftActuatorEnabled";
                     break;
                  default:
                     namespace = "StepprCommand." + actuator.getName();
                     variable = actuator.getName() + "Enabled";
                     break;
                  }

                  BooleanYoVariable actEnabled = (BooleanYoVariable) variableHolder.getVariable(namespace, variable);
                  actEnabled.set(enabled.getBooleanValue());
               }
            }
         });

         String stateNameSpace;
         String qStateVariable;
         String qdStateVariable;
         String tauStateVariable;
         switch (joint)
         {
         case LEFT_ANKLE_X:
            stateNameSpace = "Steppr.leftAnkle";
            qStateVariable = "leftAnkle_q_x";
            qdStateVariable = "leftAnkle_qd_x";
            tauStateVariable = "leftAnkle_tau_xPredictedCurrent";
            break;
         case LEFT_ANKLE_Y:
            stateNameSpace = "Steppr.leftAnkle";
            qStateVariable = "leftAnkle_q_y";
            qdStateVariable = "leftAnkle_qd_y";
            tauStateVariable = "leftAnkle_tau_yPredictedCurrent";
            break;
         case RIGHT_ANKLE_X:
            stateNameSpace = "Steppr.rightAnkle";
            qStateVariable = "rightAnkle_q_x";
            qdStateVariable = "rightAnkle_qd_x";
            tauStateVariable = "rightAnkle_tau_xPredictedCurrent";
            break;
         case RIGHT_ANKLE_Y:
            stateNameSpace = "Steppr.rightAnkle";
            qStateVariable = "rightAnkle_q_y";
            qdStateVariable = "rightAnkle_qd_y";
            tauStateVariable = "rightAnkle_tau_yPredictedCurrent";
            break;

         default:
            stateNameSpace = "Steppr." + prefix;
            qStateVariable = prefix + "_q";
            qdStateVariable = prefix + "_qd";
            tauStateVariable = prefix + "_tauPredictedCurrent";
         }
         q = (DoubleYoVariable) variableHolder.getVariable(stateNameSpace, qStateVariable);
         qd = (DoubleYoVariable) variableHolder.getVariable(stateNameSpace, qdStateVariable);
         tau = (DoubleYoVariable) variableHolder.getVariable(stateNameSpace, tauStateVariable);

         q_d = (DoubleYoVariable) variableHolder.getVariable("StepprPDJointController", prefix + "_q_d");
         qd_d = (DoubleYoVariable) variableHolder.getVariable("StepprPDJointController", prefix + "_qd_d");
         tauFF = (DoubleYoVariable) variableHolder.getVariable("StepprPDJointController", prefix + "_tau_ff");
         damping = (DoubleYoVariable) variableHolder.getVariable("StepprPDJointController", prefix + "_damping");
         kp = (DoubleYoVariable) variableHolder.getVariable("StepprPDJointController", "kp_" + prefix);
         kd = (DoubleYoVariable) variableHolder.getVariable("StepprPDJointController", "kd_" + prefix);

         tau_d = (DoubleYoVariable) variableHolder.getVariable(namespace, prefix + "TauDesired");

      }

      public void update()
      {
         selectedJoint_enabled.set(enabled.getBooleanValue());
         selectedJoint_q.set(q.getDoubleValue());
         selectedJoint_qd.set(qd.getDoubleValue());
         selectedJoint_tau.set(tau.getDoubleValue());
         selectedJoint_tau_d.set(tau_d.getDoubleValue());
         selectedJoint_q_d.set(q_d.getDoubleValue());
         selectedJoint_qd_d.set(qd_d.getDoubleValue());
         selectedJoint_kp.set(kp.getDoubleValue());
         selectedJoint_kd.set(kd.getDoubleValue());
         selectedJoint_tauFF.set(tauFF.getDoubleValue());
         selectedJoint_damping.set(damping.getDoubleValue());
      }

      public void initialize()
      {
         if(!enabled.getBooleanValue())
         {
            q_d.set(q.getDoubleValue());
            qd_d.set(0.0);
         }
      }
   }

   @Override
   public void indexChanged(int newIndex, double newTime)
   {
      if (started)
      {
         StepprJoint joint = selectedJoint.getEnumValue();
         
         for(StepprJoint stepprJoint:StepprJoint.values)
         {
            allJointVariables.get(stepprJoint).initialize();
         }
         
         allJointVariables.get(joint).update();
      }
   }

   public static void main(String[] args)
   {
      SCSVisualizer scsYoVariablesUpdatedListener = new StepprPDSliderboard(16384);
      scsYoVariablesUpdatedListener.setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener, "remote");
      client.start();

   }

}
