package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableChestTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableEndEffectorLoadBearingMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootstepDataListMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHandTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHeadTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisHeightTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisOrientationTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableStopAllTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.status.MessageStatusListener;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.status.MessageStatusListener.Status;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ControllerCommandInputManager
{
   private final int buffersCapacity = 8;

   private final SideDependentList<ConcurrentRingBuffer<ModifiableArmTrajectoryMessage>> armTrajectoryMessageBuffers = new SideDependentList<>();
   private final SideDependentList<ConcurrentRingBuffer<ModifiableHandTrajectoryMessage>> handTrajectoryMessageBuffers = new SideDependentList<>();
   private final SideDependentList<ConcurrentRingBuffer<ModifiableFootTrajectoryMessage>> footTrajectoryMessageBuffers = new SideDependentList<>();
   private final ConcurrentRingBuffer<ModifiableHeadTrajectoryMessage> headTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableChestTrajectoryMessage> chestTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiablePelvisTrajectoryMessage> pelvisTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiablePelvisOrientationTrajectoryMessage> pelvisOrientationTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiablePelvisHeightTrajectoryMessage> pelvisHeightTrajectoryMessageBuffer;

   private final ConcurrentRingBuffer<ModifiableFootstepDataListMessage> footstepDataListMessageBuffer;

   private final ConcurrentRingBuffer<ModifiableEndEffectorLoadBearingMessage> endEffectorLoadBearingMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableStopAllTrajectoryMessage> stopAllTrajectoryMessageBuffer;

   private final List<ConcurrentRingBuffer<?>> allBuffers = new ArrayList<>();

   private final List<MessageStatusListener> messageStatusListeners = new ArrayList<>();

   public ControllerCommandInputManager()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ConcurrentRingBuffer<ModifiableArmTrajectoryMessage> armTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableArmTrajectoryMessage.class), buffersCapacity);
         ConcurrentRingBuffer<ModifiableHandTrajectoryMessage> handTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableHandTrajectoryMessage.class), buffersCapacity);
         ConcurrentRingBuffer<ModifiableFootTrajectoryMessage> footTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableFootTrajectoryMessage.class), buffersCapacity);

         armTrajectoryMessageBuffers.put(robotSide, armTrajectoryMessageBuffer);
         handTrajectoryMessageBuffers.put(robotSide, handTrajectoryMessageBuffer);
         footTrajectoryMessageBuffers.put(robotSide, footTrajectoryMessageBuffer);

         allBuffers.add(armTrajectoryMessageBuffer);
         allBuffers.add(handTrajectoryMessageBuffer);
         allBuffers.add(footTrajectoryMessageBuffer);
      }
      headTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableHeadTrajectoryMessage.class), buffersCapacity);
      chestTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableChestTrajectoryMessage.class), buffersCapacity);
      pelvisTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiablePelvisTrajectoryMessage.class), buffersCapacity);
      pelvisOrientationTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiablePelvisOrientationTrajectoryMessage.class), buffersCapacity);
      pelvisHeightTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiablePelvisHeightTrajectoryMessage.class), buffersCapacity);

      footstepDataListMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableFootstepDataListMessage.class), buffersCapacity);

      endEffectorLoadBearingMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableEndEffectorLoadBearingMessage.class), buffersCapacity);
      stopAllTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableStopAllTrajectoryMessage.class), buffersCapacity);

      allBuffers.add(headTrajectoryMessageBuffer);
      allBuffers.add(chestTrajectoryMessageBuffer);
      allBuffers.add(pelvisTrajectoryMessageBuffer);
      allBuffers.add(pelvisOrientationTrajectoryMessageBuffer);
      allBuffers.add(pelvisHeightTrajectoryMessageBuffer);
      allBuffers.add(footstepDataListMessageBuffer);
      allBuffers.add(endEffectorLoadBearingMessageBuffer);
      allBuffers.add(stopAllTrajectoryMessageBuffer);
   }

   public void submitArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      ConcurrentRingBuffer<ModifiableArmTrajectoryMessage> buffer = armTrajectoryMessageBuffers.get(armTrajectoryMessage.getRobotSide());

      ModifiableArmTrajectoryMessage nextModifiableMessage = buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(armTrajectoryMessage);
      buffer.commit();
   }

   public void submitHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      ConcurrentRingBuffer<ModifiableHandTrajectoryMessage> buffer = handTrajectoryMessageBuffers.get(handTrajectoryMessage.getRobotSide());

      ModifiableHandTrajectoryMessage nextModifiableMessage = buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(handTrajectoryMessage);
      buffer.commit();
   }

   public void submitFootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      ConcurrentRingBuffer<ModifiableFootTrajectoryMessage> buffer = footTrajectoryMessageBuffers.get(footTrajectoryMessage.getRobotSide());

      ModifiableFootTrajectoryMessage nextModifiableMessage = buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(footTrajectoryMessage);
      buffer.commit();
   }

   public void submitHeadTrajectoryMessage(HeadTrajectoryMessage headTrajectoryMessage)
   {
      ModifiableHeadTrajectoryMessage nextModifiableMessage = headTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(headTrajectoryMessage);
      headTrajectoryMessageBuffer.commit();
   }

   public void submitChestTrajectoryMessage(ChestTrajectoryMessage chestTrajectoryMessage)
   {
      ModifiableChestTrajectoryMessage nextModifiableMessage = chestTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(chestTrajectoryMessage);
      chestTrajectoryMessageBuffer.commit();
   }

   public void submitPelvisTrajectoryMessage(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      ModifiablePelvisTrajectoryMessage nextModifiableMessage = pelvisTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(pelvisTrajectoryMessage);
      pelvisTrajectoryMessageBuffer.commit();
   }

   public void submitPelvisOrientationTrajectoryMessage(PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage)
   {
      ModifiablePelvisOrientationTrajectoryMessage nextModifiableMessage = pelvisOrientationTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(pelvisOrientationTrajectoryMessage);
      pelvisOrientationTrajectoryMessageBuffer.commit();
   }

   public void submitPelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage)
   {
      ModifiablePelvisHeightTrajectoryMessage nextModifiableMessage = pelvisHeightTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(pelvisHeightTrajectoryMessage);
      pelvisHeightTrajectoryMessageBuffer.commit();
   }

   public void submitWholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = wholeBodyTrajectoryMessage.getArmTrajectoryMessage(robotSide);
         if (armTrajectoryMessage != null && armTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            submitArmTrajectoryMessage(armTrajectoryMessage);
         HandTrajectoryMessage handTrajectoryMessage = wholeBodyTrajectoryMessage.getHandTrajectoryMessage(robotSide);
         if (handTrajectoryMessage != null && handTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            submitHandTrajectoryMessage(handTrajectoryMessage);
         FootTrajectoryMessage footTrajectoryMessage = wholeBodyTrajectoryMessage.getFootTrajectoryMessage(robotSide);
         if (footTrajectoryMessage != null && footTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            submitFootTrajectoryMessage(footTrajectoryMessage);
      }

      PelvisTrajectoryMessage pelvisTrajectoryMessage = wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage();
      if (pelvisTrajectoryMessage != null && pelvisTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
         submitPelvisTrajectoryMessage(pelvisTrajectoryMessage);
      ChestTrajectoryMessage chestTrajectoryMessage = wholeBodyTrajectoryMessage.getChestTrajectoryMessage();
      if (chestTrajectoryMessage != null && chestTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
         submitChestTrajectoryMessage(chestTrajectoryMessage);
   }

   public void submitFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      ModifiableFootstepDataListMessage nextModifiableMessage = footstepDataListMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(footstepDataListMessage);
      footstepDataListMessageBuffer.commit();
   }

   public void submitFootstepDataListMessage(ModifiableFootstepDataListMessage footstepDataListMessage)
   {
      ModifiableFootstepDataListMessage nextModifiableMessage = footstepDataListMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(footstepDataListMessage);
      footstepDataListMessageBuffer.commit();
   }

   public void submitEndEffectorLoadBearingMessage(EndEffectorLoadBearingMessage endEffectorLoadBearingMessage)
   {
      ModifiableEndEffectorLoadBearingMessage nextModifiableMessage = endEffectorLoadBearingMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(endEffectorLoadBearingMessage);
      endEffectorLoadBearingMessageBuffer.commit();
   }

   public void submitStopAllTrajectoryMessage(StopAllTrajectoryMessage stopAllTrajectoryMessage)
   {
      ModifiableStopAllTrajectoryMessage nextModifiableMessage = stopAllTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(stopAllTrajectoryMessage);
      stopAllTrajectoryMessageBuffer.commit();
   }

   public boolean isNewHandTrajectoryMessageAvailable(RobotSide robotSide)
   {
      return handTrajectoryMessageBuffers.get(robotSide).poll();
   }

   public boolean isNewArmTrajectoryMessageAvailable(RobotSide robotSide)
   {
      return armTrajectoryMessageBuffers.get(robotSide).poll();
   }

   public boolean isNewFootTrajectoryMessageAvailable(RobotSide robotSide)
   {
      return footTrajectoryMessageBuffers.get(robotSide).poll();
   }

   public boolean isNewHeadTrajectoryMessageAvailable()
   {
      return headTrajectoryMessageBuffer.poll();
   }

   public boolean isNewChestTrajectoryMessageAvailable()
   {
      return chestTrajectoryMessageBuffer.poll();
   }

   public boolean isNewPelvisTrajectoryMessageAvailable()
   {
      return pelvisTrajectoryMessageBuffer.poll();
   }

   public boolean isNewPelvisOrientationTrajectoryMessageAvailable()
   {
      return pelvisOrientationTrajectoryMessageBuffer.poll();
   }

   public boolean isNewPelvisHeightTrajectoryMessageAvailable()
   {
      return pelvisHeightTrajectoryMessageBuffer.poll();
   }

   public boolean isNewFootstepDataListMessageAvailable()
   {
      return footstepDataListMessageBuffer.poll();
   }

   public boolean isNewEndEffectorLoadBearingMessageAvailable()
   {
      return endEffectorLoadBearingMessageBuffer.poll();
   }

   public boolean isNewStopAllTrajectoryMessageAvailable()
   {
      return stopAllTrajectoryMessageBuffer.poll();
   }

   public ModifiableHandTrajectoryMessage pollHandTrajectoryMessage(RobotSide robotSide)
   {
      return pollNewestMessage(handTrajectoryMessageBuffers.get(robotSide));
   }

   public ModifiableArmTrajectoryMessage pollArmTrajectoryMessage(RobotSide robotSide)
   {
      return pollNewestMessage(armTrajectoryMessageBuffers.get(robotSide));
   }

   public ModifiableFootTrajectoryMessage pollFootTrajectoryMessage(RobotSide robotSide)
   {
      return pollNewestMessage(footTrajectoryMessageBuffers.get(robotSide));
   }

   public ModifiableHeadTrajectoryMessage pollHeadTrajectoryMessage()
   {
      return pollNewestMessage(headTrajectoryMessageBuffer);
   }

   public ModifiableChestTrajectoryMessage pollChestTrajectoryMessage()
   {
      return pollNewestMessage(chestTrajectoryMessageBuffer);
   }

   public ModifiablePelvisTrajectoryMessage pollPelvisTrajectoryMessage()
   {
      return pollNewestMessage(pelvisTrajectoryMessageBuffer);
   }

   public ModifiablePelvisOrientationTrajectoryMessage pollPelvisOrientationTrajectoryMessage()
   {
      return pollNewestMessage(pelvisOrientationTrajectoryMessageBuffer);
   }

   public ModifiablePelvisHeightTrajectoryMessage pollPelvisHeightTrajectoryMessage()
   {
      return pollNewestMessage(pelvisHeightTrajectoryMessageBuffer);
   }

   public ModifiableFootstepDataListMessage pollFootstepDataListMessage()
   {
      return pollNewestMessage(footstepDataListMessageBuffer);
   }

   public ModifiableEndEffectorLoadBearingMessage pollEndEffectorLoadBearingMessage()
   {
      return pollNewestMessage(endEffectorLoadBearingMessageBuffer);
   }

   public ModifiableStopAllTrajectoryMessage pollStopAllTrajectoryMessage()
   {
      return pollNewestMessage(stopAllTrajectoryMessageBuffer);
   }
   
   public void reportMessageStatus(Status status, Class<? extends IHMCRosApiMessage<?>> messageClass)
   {
      reportMessageStatus(status, messageClass, Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public void reportMessageStatus(Status status, Class<? extends IHMCRosApiMessage<?>> messageClass, long id)
   {
      for (int i = 0; i < messageStatusListeners.size(); i++)
      {
         messageStatusListeners.get(i).receivedNewMessageStatus(status, messageClass, id);
      }
   }

   public void subscribeToMessageStatus(MessageStatusListener messageStatusListener)
   {
      messageStatusListeners.add(messageStatusListener);
   }

   public void clearManipulationMessagesInQueue()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handTrajectoryMessageBuffers.get(robotSide).flush();
         armTrajectoryMessageBuffers.get(robotSide).flush();
      }
   }

   public void clearMessagesInQueue()
   {
      for (int i = 0; i < allBuffers.size(); i++)
         allBuffers.get(i).flush();
   }

   private static <T> T pollNewestMessage(ConcurrentRingBuffer<T> buffer)
   {
      if (buffer.poll())
      {
         T newestMessage = null;
         T message;
         while ((message = buffer.read()) != null)
         {
            newestMessage = message;
         }
         buffer.flush();
         return newestMessage;
      }
      return null;
   }

   private static <U> Builder<U> createBuilderWithEmptyConstructor(Class<U> clazz)
   {
      final Constructor<U> emptyConstructor;
      // Trying to get an empty constructor from clazz
      try
      {
         emptyConstructor = clazz.getConstructor();
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         throw new RuntimeException("Could not find a visible empty constructor in the class: " + clazz.getSimpleName());
      }

      Builder<U> builder = new Builder<U>()
      {
         @Override
         public U newInstance()
         {
            U newInstance = null;

            try
            {
               newInstance = emptyConstructor.newInstance();
            }
            catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
            {
               e.printStackTrace();
               throw new RuntimeException(
                     "Something went wrong the empty constructor implemented in the class: " + emptyConstructor.getDeclaringClass().getSimpleName());
            }

            return newInstance;
         }
      };
      return builder;
   }
}
