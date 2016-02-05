package object_manipulation_msgs;

public interface ReactiveGraspAction extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/ReactiveGraspAction";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nReactiveGraspActionGoal action_goal\nReactiveGraspActionResult action_result\nReactiveGraspActionFeedback action_feedback\n";
  object_manipulation_msgs.ReactiveGraspActionGoal getActionGoal();
  void setActionGoal(object_manipulation_msgs.ReactiveGraspActionGoal value);
  object_manipulation_msgs.ReactiveGraspActionResult getActionResult();
  void setActionResult(object_manipulation_msgs.ReactiveGraspActionResult value);
  object_manipulation_msgs.ReactiveGraspActionFeedback getActionFeedback();
  void setActionFeedback(object_manipulation_msgs.ReactiveGraspActionFeedback value);
}
