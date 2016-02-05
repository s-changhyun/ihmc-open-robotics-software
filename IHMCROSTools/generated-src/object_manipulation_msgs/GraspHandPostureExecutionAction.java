package object_manipulation_msgs;

public interface GraspHandPostureExecutionAction extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/GraspHandPostureExecutionAction";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nGraspHandPostureExecutionActionGoal action_goal\nGraspHandPostureExecutionActionResult action_result\nGraspHandPostureExecutionActionFeedback action_feedback\n";
  object_manipulation_msgs.GraspHandPostureExecutionActionGoal getActionGoal();
  void setActionGoal(object_manipulation_msgs.GraspHandPostureExecutionActionGoal value);
  object_manipulation_msgs.GraspHandPostureExecutionActionResult getActionResult();
  void setActionResult(object_manipulation_msgs.GraspHandPostureExecutionActionResult value);
  object_manipulation_msgs.GraspHandPostureExecutionActionFeedback getActionFeedback();
  void setActionFeedback(object_manipulation_msgs.GraspHandPostureExecutionActionFeedback value);
}
