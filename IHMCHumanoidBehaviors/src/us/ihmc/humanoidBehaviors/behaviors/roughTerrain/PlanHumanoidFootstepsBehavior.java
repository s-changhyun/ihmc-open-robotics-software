package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.packets.UIPositionCheckerPacket;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PlanHumanoidFootstepsBehavior extends AbstractBehavior
{
   private final String prefix = "planFootsteps";

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   private final ConcurrentListeningQueue<PlanarRegionsListMessage> planarRegionsListQueue = new ConcurrentListeningQueue<>(10);

   private final IntegerYoVariable planarRegionsListCount = new IntegerYoVariable(prefix + "PlanarRegionsListCount", registry);
   private final BooleanYoVariable foundPlan = new BooleanYoVariable(prefix + "FoundPlan", registry);
   private final BooleanYoVariable requestedPlanarRegion = new BooleanYoVariable(prefix + "RequestedPlanarRegion", registry);
   private final DoubleYoVariable shorterGoalLength = new DoubleYoVariable(prefix + "ShorterGoalLength", registry);

   private final EnumYoVariable<RobotSide> nextSideToSwing;

   private final PlanarRegionBipedalFootstepPlanner footstepPlanner;
   private FootstepPlan plan = null;

   private final YoFramePose footstepPlannerInitialStepPose;
   private final YoFramePose footstepPlannerGoalPose;

   private final FootstepPlannerGoal footstepPlannerGoal = new FootstepPlannerGoal();
   private final FramePose goalPose = new FramePose();

   private final FramePose leftFootPose = new FramePose();
   private final FramePose rightFootPose = new FramePose();
   private final FramePose tempStanceFootPose = new FramePose();
   private final FramePose tempFirstFootstepPose = new FramePose();
   private final Point3D tempFootstepPosePosition = new Point3D();
   private final Quaternion tempFirstFootstepPoseOrientation = new Quaternion();
   private final YoStopwatch plannerTimer;

   public PlanHumanoidFootstepsBehavior(DoubleYoVariable yoTime, CommunicationBridge behaviorCommunicationBridge, FullHumanoidRobotModel fullRobotModel,
                                        HumanoidReferenceFrames referenceFrames)
   {
      super(PlanHumanoidFootstepsBehavior.class.getSimpleName(), behaviorCommunicationBridge);

      shorterGoalLength.set(1.5);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      footstepPlanner = createFootstepPlanner();

      nextSideToSwing = new EnumYoVariable<>("nextSideToSwing", registry, RobotSide.class);
      nextSideToSwing.set(RobotSide.LEFT);

      plannerTimer = new YoStopwatch(yoTime);
      plannerTimer.start();

      footstepPlannerGoalPose = new YoFramePose(prefix + "FootstepGoalPose", ReferenceFrame.getWorldFrame(), registry);
      footstepPlannerInitialStepPose = new YoFramePose(prefix + "InitialStepPose", ReferenceFrame.getWorldFrame(), registry);

      attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);

      requestedPlanarRegion.set(false);
   }

   private PlanarRegionBipedalFootstepPlanner createFootstepPlanner()
   {
      BipedalFootstepPlannerParameters parameters = new BipedalFootstepPlannerParameters(registry);

      parameters.setMaximumStepReach(0.65); //0.55); //(0.4);
      parameters.setMaximumStepZ(0.25); //0.4); //0.25);

      // Atlas has ankle pitch range of motion limits, which hit when taking steps forward and down. Similar to a human.
      // Whereas a human gets on its toes nicely to avoid the limits, this is challenging with a robot.
      // So for now, have really conservative forward and down limits on height.
      parameters.setMaximumStepXWhenForwardAndDown(0.2);
      parameters.setMaximumStepWidth(0.4);
      parameters.setMaximumStepZWhenForwardAndDown(0.10);

      parameters.setMaximumStepYaw(0.15); //0.25);
      parameters.setMinimumStepWidth(0.15);
      parameters.setMinimumFootholdPercent(0.95);

      parameters.setWiggleInsideDelta(0.02); //0.08);
      parameters.setMaximumXYWiggleDistance(1.0);
      parameters.setMaximumYawWiggle(0.1);

      parameters.setRejectIfCannotFullyWiggleInside(true);

      double idealFootstepLength = 0.45; //0.3; //0.4;
      double idealFootstepWidth = 0.26; //0.2; //0.25;
      parameters.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      PlanarRegionBipedalFootstepPlanner planner = new PlanarRegionBipedalFootstepPlanner(parameters, registry);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      planner.setMaximumNumberOfNodesToExpand(500);
      return planner;
   }

   public void createAndAttachSCSListenerToPlanner()
   {
      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = footstepPlanner.getFootPolygonsInSoleFrame();
      PlanarRegionBipedalFootstepPlannerVisualizer listener = PlanarRegionBipedalFootstepPlannerVisualizerFactory.createWithSimulationConstructionSet(0.01,
                                                                                                                                                      footPolygonsInSoleFrame);


      SimulationConstructionSet scs = (SimulationConstructionSet) listener.getTickAndUpdatable();
      //      scs.setCameraFix(-6.0, 0.0, 0.0);
      //      scs.setCameraPosition(-11.0, 0.0, 8.0);

      footstepPlanner.setBipedalFootstepPlannerListener(listener);
   }

   public void createAndAttachYoVariableServerListenerToPlanner(LogModelProvider logModelProvider, FullRobotModel fullRobotModel)
   {
      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = footstepPlanner.getFootPolygonsInSoleFrame();
      PlanarRegionBipedalFootstepPlannerVisualizer listener = PlanarRegionBipedalFootstepPlannerVisualizerFactory.createWithYoVariableServer(0.01,
                                                                                                                                             fullRobotModel,
                                                                                                                                             logModelProvider,
                                                                                                                                             footPolygonsInSoleFrame, "Behavior_");

      footstepPlanner.setBipedalFootstepPlannerListener(listener);
   }

   public void setGoalPoseAndFirstSwingSide(FramePose goalPose, RobotSide swingSide)
   {
      this.nextSideToSwing.set(swingSide);

      foundPlan.set(false);
      this.plan = null;
      this.goalPose.set(goalPose);
   }

   public FootstepDataListMessage getFootstepDataListMessageForPlan(int maxNumberOfStepsToTake, double swingTime, double transferTime)
   {
      if (plan == null)
         return null;

      FootstepDataListMessage footstepDataListMessage = createFootstepDataListFromPlan(plan, maxNumberOfStepsToTake, swingTime, transferTime);
      return footstepDataListMessage;
   }

   private int failIndex = 0;

   @Override
   public void doControl()
   {
      if (plannerTimer.totalElapsed() < 0.5)
         return;

      if (!requestedPlanarRegion.getBooleanValue() || (plannerTimer.totalElapsed() > 5.0))
      {
         clearAndRequestPlanarRegionsList();
         requestedPlanarRegion.set(true);
      }

      boolean planarRegionsListIsAvailable = updatePlannerIfPlanarRegionsListIsAvailable();
      if (!planarRegionsListIsAvailable)
      {
         return;
      }

      setGoalAndInitialStanceFootToBeClosestToGoal(goalPose);

      footstepPlanner.plan();
      plan = footstepPlanner.getPlan();

      plannerTimer.reset();
      requestedPlanarRegion.set(false);

      if (plan == null)
      {
         sendTextToSpeechPacket("No Plan was found! " + failIndex++);
         this.nextSideToSwing.set(this.nextSideToSwing.getEnumValue().getOppositeSide());
         return;
      }

      failIndex = 0;

      sendTextToSpeechPacket("Found plan!");
      foundPlan.set(true);
   }

   private void sendTextToSpeechPacket(String message)
   {
      TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket(message);
      textToSpeechPacket.setbeep(false);
      sendPacketToUI(textToSpeechPacket);
   }

   private void clearAndRequestPlanarRegionsList()
   {
      planarRegionsListQueue.getLatestPacket();

      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(RequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      sendPacket(requestPlanarRegionsListMessage);
   }

   private boolean updatePlannerIfPlanarRegionsListIsAvailable()
   {
      if (planarRegionsListQueue.isNewPacketAvailable())
      {
         planarRegionsListCount.increment();

         PlanarRegionsListMessage planarRegionsListMessage = planarRegionsListQueue.getLatestPacket();
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         footstepPlanner.setPlanarRegions(planarRegionsList);
         return true;
      }

      return false;
   }

   private void setGoalAndInitialStanceFootToBeClosestToGoal(FramePose goalPose)
   {
      //      sendPacketToUI(new UIPositionCheckerPacket(goalPose.getFramePointCopy().getPoint(), goalPose.getFrameOrientationCopy().getQuaternion()));

      leftFootPose.setToZero(referenceFrames.getSoleFrame(RobotSide.LEFT));
      rightFootPose.setToZero(referenceFrames.getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D temp = new Point3D();
      Point3D pointBetweenFeet = new Point3D();
      Point3D goalPosition = new Point3D();
      Point3D shorterGoalPosition = new Point3D();
      Vector3D vectorFromFeetToGoal = new Vector3D();

      leftFootPose.getPosition(temp);
      pointBetweenFeet.set(temp);
      rightFootPose.getPosition(temp);
      pointBetweenFeet.add(temp);
      pointBetweenFeet.scale(0.5);

      goalPose.getPosition(goalPosition);
      vectorFromFeetToGoal.sub(goalPosition, pointBetweenFeet);

      if (vectorFromFeetToGoal.length() > shorterGoalLength.getDoubleValue())
      {
         vectorFromFeetToGoal.scale(shorterGoalLength.getDoubleValue() / vectorFromFeetToGoal.length());
      }
      shorterGoalPosition.set(pointBetweenFeet);
      shorterGoalPosition.add(vectorFromFeetToGoal);
      goalPose.setPosition(shorterGoalPosition);

      double headingFromFeetToGoal = Math.atan2(vectorFromFeetToGoal.getY(), vectorFromFeetToGoal.getX());
      AxisAngle goalOrientation = new AxisAngle(0.0, 0.0, 1.0, headingFromFeetToGoal);
      goalPose.setOrientation(goalOrientation);

      RobotSide stanceSide;
      //      if (currentlySwingingFoot.getEnumValue() != null)
      //      {
      //         stanceSide = currentlySwingingFoot.getEnumValue();
      //
      //         this.desiredFootStatusPoses.get(stanceSide).getFramePose(tempStanceFootPose);
      //         goalPose.setZ(tempStanceFootPose.getZ());
      //      }
      //
      //      else
      {
         stanceSide = nextSideToSwing.getEnumValue().getOppositeSide();

         if (stanceSide == RobotSide.LEFT)
         {
            tempStanceFootPose.set(leftFootPose);
            goalPose.setZ(leftFootPose.getZ());
         }
         else
         {
            tempStanceFootPose.set(rightFootPose);
            goalPose.setZ(rightFootPose.getZ());
         }
      }

      //      sendTextToSpeechPacket("Planning footsteps from " + tempStanceFootPose + " to " + goalPose);
      //      sendTextToSpeechPacket("Planning footsteps to the fiducial");
      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);

      // For now, just get close to the Fiducial, don't need to get exactly on it.
      Point2D xyGoal = new Point2D();
      xyGoal.setX(goalPose.getX());
      xyGoal.setY(goalPose.getY());
      double distanceFromXYGoal = 1.0;
      footstepPlannerGoal.setXYGoal(xyGoal, distanceFromXYGoal);
      //      footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.CLOSE_TO_XY_POSITION);

      sendPacketToUI(new UIPositionCheckerPacket(new Point3D(xyGoal.getX(), xyGoal.getY(), leftFootPose.getZ()), new Quaternion()));

      footstepPlanner.setGoal(footstepPlannerGoal);

      footstepPlanner.setInitialStanceFoot(tempStanceFootPose, stanceSide);

      footstepPlannerGoalPose.set(goalPose);
      footstepPlannerInitialStepPose.set(tempStanceFootPose);
   }

   private FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan plan, int maxNumberOfStepsToTake, double swingTime, double transferTime)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingTime(swingTime);
      footstepDataListMessage.setDefaultTransferTime(transferTime);
      int lastStepIndex = Math.min(maxNumberOfStepsToTake + 1, plan.getNumberOfSteps());
      for (int i = 1; i < lastStepIndex; i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         footstep.getSoleFramePose(tempFirstFootstepPose);
         tempFirstFootstepPose.getPosition(tempFootstepPosePosition);
         tempFirstFootstepPose.getOrientation(tempFirstFootstepPoseOrientation);

         FootstepDataMessage firstFootstepMessage = new FootstepDataMessage(footstep.getRobotSide(), new Point3D(tempFootstepPosePosition),
                                                                            new Quaternion(tempFirstFootstepPoseOrientation));
         firstFootstepMessage.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

         footstepDataListMessage.add(firstFootstepMessage);
      }

      footstepDataListMessage.setExecutionMode(ExecutionMode.OVERRIDE);
      return footstepDataListMessage;
   }

   @Override
   public void onBehaviorEntered()
   {
      plannerTimer.start();
      plannerTimer.reset();
   }

   @Override
   public boolean isDone()
   {
      return foundPlan.getBooleanValue();
   }

   private static ConvexPolygon2d createDefaultFootPolygon()
   {
      //TODO: Get this from the robot model itself.
      double footLength = 0.26;
      double footWidth = 0.18;

      ConvexPolygon2d footPolygon = new ConvexPolygon2d();
      footPolygon.addVertex(footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      footPolygon.update();

      return footPolygon;
   }

   private static SideDependentList<ConvexPolygon2d> createDefaultFootPolygons()
   {
      SideDependentList<ConvexPolygon2d> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         footPolygons.put(side, createDefaultFootPolygon());
      return footPolygons;
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
   }
}
