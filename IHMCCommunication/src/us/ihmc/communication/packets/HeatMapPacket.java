package us.ihmc.communication.packets;

import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.ArrayTools;

import java.util.Arrays;
import java.util.Random;

/**
 *
 */
public class HeatMapPacket extends Packet<HeatMapPacket>
{
   public float[] data;
   public int width, height;

   public HeatMapPacket()
   {
   }

   public HeatMapPacket(HeatMapPacket other)
   {
      this.data = Arrays.copyOf(other.data, other.data.length);
      this.width = other.width;
      this.height = other.height;
   }

   public HeatMapPacket(Random random)
   {
      this.height = RandomTools.generateRandomInt(random, -100, 100);
      this.width = RandomTools.generateRandomInt(random, -100, 100);
      data = new float[this.height * this.width];

      for(int i = 0; i < data.length; i++)
      {
         data[i] = RandomTools.generateRandomFloatInRange(random, 0.0f, 1.0f);
      }
   }

   @Override public boolean epsilonEquals(HeatMapPacket other, double epsilon)
   {
      boolean widthEquals = other.width == this.width;
      boolean heightEquals = other.height == this.height;

      return widthEquals && heightEquals && ArrayTools.deltaEquals(this.data, other.data, (float) epsilon);
   }
}