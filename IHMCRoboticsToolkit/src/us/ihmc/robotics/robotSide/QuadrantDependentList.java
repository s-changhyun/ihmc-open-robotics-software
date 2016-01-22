package us.ihmc.robotics.robotSide;

import static us.ihmc.robotics.robotSide.RobotQuadrant.FRONT_LEFT_ORDINAL;
import static us.ihmc.robotics.robotSide.RobotQuadrant.FRONT_RIGHT_ORDINAL;
import static us.ihmc.robotics.robotSide.RobotQuadrant.HIND_LEFT_ORDINAL;
import static us.ihmc.robotics.robotSide.RobotQuadrant.HIND_RIGHT_ORDINAL;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;

@SuppressWarnings("unchecked")
public class QuadrantDependentList<V>
{ 
   private final V[] elements = (V[]) new Object[4];
   private int size = 0;
   
   private final RobotQuadrant[][] quadrantArrays = new RobotQuadrant[5][];
   {
      quadrantArrays[0] = new RobotQuadrant[0];
      quadrantArrays[1] = new RobotQuadrant[1];
      quadrantArrays[2] = new RobotQuadrant[2];
      quadrantArrays[3] = new RobotQuadrant[3];
      quadrantArrays[4] = RobotQuadrant.values;
   }

   public QuadrantDependentList()
   {
      super();
   }
   
   public QuadrantDependentList(V frontLeftObject, V frontRightObject, V hindLeftObject, V hindRightObject)
   {
      super();
      
      elements[FRONT_LEFT_ORDINAL] = frontLeftObject;
      elements[FRONT_RIGHT_ORDINAL] = frontRightObject;
      elements[HIND_LEFT_ORDINAL] = hindLeftObject;
      elements[HIND_RIGHT_ORDINAL] = hindRightObject;
      size = 4;
   }
   
   public V get(RobotQuadrant key)
   {
      return elements[key.ordinal()];
   }
   
   public void set(RobotQuadrant robotQuadrant, V element)
   {
      V existingElement = get(robotQuadrant);
      if (existingElement == element)
         return;
      
      if (element == null)
         --size;
      else if (existingElement == null)
         ++size;
      
      elements[robotQuadrant.ordinal()] = element;
      
      packQuadrantArray();
   }
   
   public V remove(RobotQuadrant robotQuadrant)
   {
      V element = get(robotQuadrant);
      
      if (element != null)
         --size;
      
      elements[robotQuadrant.ordinal()] = null;
      
      packQuadrantArray();
      
      return element;
   }
   
   public void clear()
   {
      elements[FRONT_LEFT_ORDINAL] = null;
      elements[FRONT_RIGHT_ORDINAL] = null;
      elements[HIND_LEFT_ORDINAL] = null;
      elements[HIND_RIGHT_ORDINAL] = null;
      size = 0;
   }
   
   private void packQuadrantArray()
   {
      if (size == 4)
         return;
      
      for (int i = 0, j = 0; i < RobotQuadrant.values.length; i++)
      {
         if (containsKey(RobotQuadrant.values[i]))
         {
            quadrantArrays[size][j++] = RobotQuadrant.values[i];
         }
      }
   }
   
   public boolean containsKey(RobotQuadrant robotQuadrant)
   {
      return elements[robotQuadrant.ordinal()] != null;
   }
   
   public int size()
   {
      return size;
   }
   
   public RobotQuadrant[] quadrants()
   {
      return quadrantArrays[size];
   }
   
   public static <K extends Enum<K>, V> QuadrantDependentList<EnumMap<K, V>> createListOfEnumMaps(Class<K> keyType)
   {
      return new QuadrantDependentList<EnumMap<K, V>>(new EnumMap<K, V>(keyType), new EnumMap<K, V>(keyType),new EnumMap<K, V>(keyType),new EnumMap<K, V>(keyType));
   }

   public static <K, V> QuadrantDependentList<Map<K, V>> createListOfHashMaps()
   {
      return new QuadrantDependentList<Map<K, V>>(new LinkedHashMap<K, V>(), new LinkedHashMap<K, V>(),new LinkedHashMap<K, V>(),new LinkedHashMap<K, V>());
   }
   
   public static <V> QuadrantDependentList<ArrayList<V>> createListOfArrayLists()
   {
      return new QuadrantDependentList<ArrayList<V>>(new ArrayList<V>(), new ArrayList<V>(), new ArrayList<V>(), new ArrayList<V>());
   }
}
