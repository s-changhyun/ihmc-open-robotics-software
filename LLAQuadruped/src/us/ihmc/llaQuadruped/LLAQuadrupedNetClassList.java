package us.ihmc.llaQuadruped;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

import us.ihmc.euclid.tuple3D.Vector3D;

import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.quadrupedRobotics.communication.packets.BodyAngularRatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComVelocityPacket;
import us.ihmc.quadrupedRobotics.communication.packets.PlanarVelocityPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerStatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedNeckJointPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedTimedStepPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedXGaitSettingsPacket;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.robotics.dataStructures.parameter.BooleanParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.communication.packets.ParameterListPacket;
import us.ihmc.communication.packets.RequestParameterListPacket;
import us.ihmc.communication.packets.SetBooleanParameterPacket;
import us.ihmc.communication.packets.SetDoubleArrayParameterPacket;
import us.ihmc.communication.packets.SetDoubleParameterPacket;
import us.ihmc.communication.packets.SetStringParameterPacket;
import us.ihmc.robotics.dataStructures.parameter.StringParameter;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class LLAQuadrupedNetClassList extends IHMCCommunicationKryoNetClassList
{
   public LLAQuadrupedNetClassList()
   {
      super();
      registerPacketClass(Packet.class);

      registerPacketClass(QuadrupedNeckJointPositionPacket.class);
      registerPacketField(QuadrupedJointName.class);
      registerPacketField(QuadrupedJointName[].class);
      registerPacketField(double[].class);

      registerPacketClass(QuadrupedTimedStepPacket.class);
      registerPacketField(ArrayList.class);
      registerPacketField(QuadrupedTimedStep.class);

      registerPacketField(Point2D32.class);

      registerPacketField(String[].class);
      registerPacketField(Point3D[].class);

      registerPacketClass(ComPositionPacket.class);
      registerPacketField(Point3D.class);

      registerPacketClass(ComVelocityPacket.class);
      registerPacketField(Vector3D.class);

      registerPacketClass(BodyOrientationPacket.class);
      registerPacketField(Quaternion.class);

      registerPacketClass(BodyAngularRatePacket.class);
      registerPacketField(Vector3D.class);

      registerPacketClass(PlanarVelocityPacket.class);
      registerPacketField(Vector3D.class);

      registerPacketClass(QuadrupedForceControllerEventPacket.class);
      registerPacketField(QuadrupedForceControllerRequestedEvent.class);

      registerPacketClass(QuadrupedForceControllerStatePacket.class);
      registerPacketField(QuadrupedForceControllerState.class);

      registerPacketClass(QuadrupedXGaitSettingsPacket.class);
      registerPacketField(QuadrupedXGaitSettings.class);

      registerPacketClass(SetBooleanParameterPacket.class);
      registerPacketField(String.class);
      registerPacketField(boolean.class);

      registerPacketClass(SetDoubleArrayParameterPacket.class);
      registerPacketField(String.class);
      registerPacketField(double[].class);

      registerPacketClass(SetDoubleParameterPacket.class);
      registerPacketField(String.class);
      registerPacketField(double.class);

      registerPacketClass(SetStringParameterPacket.class);
      registerPacketField(String.class);

      registerPacketClass(QuadrupedTimedStepPacket.class);
      registerPacketField(ArrayList.class);
      registerPacketField(QuadrupedTimedStep.class);
      registerPacketField(Point3D.class);
      registerPacketField(RobotQuadrant.class);
      registerPacketField(TimeInterval.class);

      registerPacketClass(RequestParameterListPacket.class);
      registerPacketClass(ParameterListPacket.class);
      registerPacketField(BooleanParameter.class);
      registerPacketField(DoubleArrayParameter .class);
      registerPacketField(DoubleParameter.class);
      registerPacketField(StringParameter.class);
   }
}
