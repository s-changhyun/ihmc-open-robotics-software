package pr2_mechanism_msgs;

public interface SwitchControllerActionResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_msgs/SwitchControllerActionResult";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nSwitchControllerResult result\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalStatus getStatus();
  void setStatus(actionlib_msgs.GoalStatus value);
  pr2_mechanism_msgs.SwitchControllerResult getResult();
  void setResult(pr2_mechanism_msgs.SwitchControllerResult value);
}
