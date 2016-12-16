package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.util.EnumMap;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.FootstepPlannerForBehaviorsHelper;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.PlanarRegionBipedalFootstepPlannerVisualizerFactory;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessageConverter;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class FootstepPlanningToolboxController extends ToolboxController
{
   private final boolean visualize = true;

   private enum Planners
   {
      PLANAR_REGION_BIPEDAL,
      PLAN_THEN_SNAP
   }
   private final EnumYoVariable<Planners> activePlanner = new EnumYoVariable<>("activePlanner", registry, Planners.class);
   private final EnumMap<Planners, FootstepPlanner> plannerMap = new EnumMap<>(Planners.class);

   private final AtomicReference<FootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<FootstepPlanningRequestPacket>(null);
   private final AtomicReference<PlanarRegionsListMessage> latestPlanarRegionsReference = new AtomicReference<PlanarRegionsListMessage>(null);

   private final BooleanYoVariable usePlanarRegions = new BooleanYoVariable("usePlanarRegions", registry);
   private final BooleanYoVariable isDone = new BooleanYoVariable("isDone", registry);
   private final BooleanYoVariable requestedPlanarRegions = new BooleanYoVariable("RequestedPlanarRegions", registry);
   private final DoubleYoVariable toolboxTime = new DoubleYoVariable("ToolboxTime", registry);
   private final DoubleYoVariable timeReceivedPlanarRegion = new DoubleYoVariable("timeReceivedPlanarRegion", registry);

   private final PacketCommunicator packetCommunicator;
   private long plannerCount = 0;
   private double dt;

   public FootstepPlanningToolboxController(RobotContactPointParameters contactPointParameters, StatusMessageOutputManager statusOutputManager, PacketCommunicator packetCommunicator, YoVariableRegistry parentRegistry,
         double dt)
   {
      super(statusOutputManager, parentRegistry);
      this.packetCommunicator = packetCommunicator;
      this.dt = dt;
      packetCommunicator.attachListener(PlanarRegionsListMessage.class, createPlanarRegionsConsumer());

//      SideDependentList<ConvexPolygon2d> footPolygons = new SideDependentList<>();
//      for (RobotSide side : RobotSide.values)
//         footPolygons.set(side, new ConvexPolygon2d(contactPointParameters.getFootContactPoints().get(side)));

      SideDependentList<ConvexPolygon2d> planningPolygonsInSoleFrame = FootstepPlannerForBehaviorsHelper.createDefaultFootPolygonsForAnytimePlannerAndPlannerToolbox(contactPointParameters);
      SideDependentList<ConvexPolygon2d> controllerPolygonsInSoleFrame = FootstepPlannerForBehaviorsHelper.createDefaultFootPolygons(contactPointParameters, 1.0, 1.0);

      plannerMap.put(Planners.PLANAR_REGION_BIPEDAL, createPlanarRegionBipedalPlanner(planningPolygonsInSoleFrame, controllerPolygonsInSoleFrame));
      plannerMap.put(Planners.PLAN_THEN_SNAP, new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), planningPolygonsInSoleFrame));
      activePlanner.set(Planners.PLANAR_REGION_BIPEDAL);

      usePlanarRegions.set(true);
      isDone.set(true);
   }

   private PlanarRegionBipedalFootstepPlanner createPlanarRegionBipedalPlanner(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame, SideDependentList<ConvexPolygon2d> controllerPolygonsInSoleFrame)
   {
      BipedalFootstepPlannerParameters footstepPlanningParameters = new BipedalFootstepPlannerParameters(registry);
      FootstepPlannerForBehaviorsHelper.setPlannerParametersForAnytimePlannerAndPlannerToolbox(footstepPlanningParameters);

      PlanarRegionBipedalFootstepPlanner footstepPlanner = new PlanarRegionBipedalFootstepPlanner(footstepPlanningParameters, registry);
      footstepPlanner.setFeetPolygons(footPolygonsInSoleFrame, controllerPolygonsInSoleFrame);
      footstepPlanner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      footstepPlanner.setExitAfterInitialSolution(false);
      footstepPlanner.setTimeout(5.0);

      if (visualize)
      {
         PlanarRegionBipedalFootstepPlannerVisualizer listener = PlanarRegionBipedalFootstepPlannerVisualizerFactory.createWithYoVariableServer(0.01, null,
               null, footPolygonsInSoleFrame, "Toolbox_");
         footstepPlanner.setBipedalFootstepPlannerListener(listener);
      }

      return footstepPlanner;
   }

   @Override
   protected void updateInternal()
   {
      toolboxTime.add(dt);
      if (toolboxTime.getDoubleValue() > 10.0)
      {
         reportMessage(packResult(null, FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION));
         isDone.set(true);
         return;
      }

      FootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      if (usePlanarRegions.getBooleanValue())
      {
         if (!requestAndWaitForPlanarRegions(planner))
            return;
      }
      else
      {
         planner.setPlanarRegions(null);
      }

      sendMessageToUI("Starting To Plan: " + plannerCount + ", " + activePlanner.getEnumValue().toString());

      FootstepPlanningResult status = planner.plan();
      FootstepPlan footstepPlan = planner.getPlan();

      sendMessageToUI("Result: " + plannerCount + ", " + status.toString());
      plannerCount++;

      reportMessage(packResult(footstepPlan, status));
      isDone.set(true);
   }

   private boolean requestAndWaitForPlanarRegions(FootstepPlanner planner)
   {
      if (!requestedPlanarRegions.getBooleanValue())
         requestPlanarRegions();

      PlanarRegionsListMessage planarRegionsMessage = latestPlanarRegionsReference.getAndSet(null);
      if (planarRegionsMessage == null)
         return false;

      timeReceivedPlanarRegion.set(toolboxTime.getDoubleValue());
      PlanarRegionsList planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsMessage);
      planner.setPlanarRegions(planarRegions);

      return true;
   }

   private void requestPlanarRegions()
   {
      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(RequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      packetCommunicator.send(requestPlanarRegionsListMessage);
      latestPlanarRegionsReference.set(null);
      requestedPlanarRegions.set(true);
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);
      requestedPlanarRegions.set(false);
      toolboxTime.set(0.0);
      timeReceivedPlanarRegion.set(0.0);

      FootstepPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;

      usePlanarRegions.set(!request.assumeFlatGround);

      FramePose initialStancePose = new FramePose(ReferenceFrame.getWorldFrame());
      initialStancePose.setPosition(new Point3d(request.stanceFootPositionInWorld));
      initialStancePose.setOrientation(new Quat4d(request.stanceFootOrientationInWorld));

      FramePose goalPose = new FramePose(ReferenceFrame.getWorldFrame());
      goalPose.setPosition(new Point3d(request.goalPositionInWorld));
      goalPose.setOrientation(new Quat4d(request.goalOrientationInWorld));

      FootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());
      planner.setInitialStanceFoot(initialStancePose, request.initialStanceSide);

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);
      planner.setGoal(goal);

      return true;
   }

   public PacketConsumer<FootstepPlanningRequestPacket> createRequestConsumer()
   {
      return new PacketConsumer<FootstepPlanningRequestPacket>()
      {
         @Override
         public void receivedPacket(FootstepPlanningRequestPacket packet)
         {
            if (packet == null)
               return;
            latestRequestReference.set(packet);
         }
      };
   }

   public PacketConsumer<PlanarRegionsListMessage> createPlanarRegionsConsumer()
   {
      return new PacketConsumer<PlanarRegionsListMessage>()
      {
         @Override
         public void receivedPacket(PlanarRegionsListMessage packet)
         {
            if (packet == null)
               return;
            latestPlanarRegionsReference.set(packet);
         }
      };
   }

   private void sendMessageToUI(String message)
   {
      TextToSpeechPacket packet = new TextToSpeechPacket(message);
      packet.setDestination(PacketDestination.UI);
      packetCommunicator.send(packet);
   }

   @Override
   protected boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private FootstepPlanningToolboxOutputStatus packResult(FootstepPlan footstepPlan, FootstepPlanningResult status)
   {
      if (footstepPlan == null)
         footstepPlan = new FootstepPlan();

      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();
      result.footstepDataList = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, 0.0, 0.0, ExecutionMode.OVERRIDE);
      result.planningResult = status;
      return result;
   }

}
