package control_msgs;

public interface PointHeadActionAction extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "control_msgs/PointHeadActionAction";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nPointHeadActionActionGoal action_goal\nPointHeadActionActionResult action_result\nPointHeadActionActionFeedback action_feedback\n";
  control_msgs.PointHeadActionActionGoal getActionGoal();
  void setActionGoal(control_msgs.PointHeadActionActionGoal value);
  control_msgs.PointHeadActionActionResult getActionResult();
  void setActionResult(control_msgs.PointHeadActionActionResult value);
  control_msgs.PointHeadActionActionFeedback getActionFeedback();
  void setActionFeedback(control_msgs.PointHeadActionActionFeedback value);
}
