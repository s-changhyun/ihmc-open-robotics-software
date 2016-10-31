package us.ihmc.valkyrie.parameters;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

import gnu.trove.map.hash.TObjectDoubleHashMap;

public class ValkyrieLocalizationParameters
{
   private static enum keys {DEADBAND_X, DEADBAND_Y, DEADBAND_Z, YAW_DEADBAND_IN_DEGREES};
   private static final String defaultConfigFile = "resources/us/ihmc/valkyrie/configuration/parameters/localization.ini";
   private static final TObjectDoubleHashMap<keys> deadbands = new TObjectDoubleHashMap<keys>();

   static
   {
      File file = new File(System.getProperty("jointOffsetFile", defaultConfigFile));
      System.out.println("Loading Valkyrie Localization Parameters from " + file.getAbsolutePath());

      if (file.exists() && file.isFile())
      {
         try
         {
            Properties properties = new Properties();
            FileInputStream stream = new FileInputStream(file);
            properties.load(stream);

            for (keys key : keys.values())
            {
               if (properties.containsKey(key.toString()))
               {
                  String value = properties.getProperty(key.toString());
                  deadbands.put(key, Double.parseDouble(value));
               }
            }
            stream.close();
         }
         catch (IOException e)
         {
            throw new RuntimeException("Valkyrie Localization Parameters file " + file.getAbsolutePath() + " cannot be loaded. ", e);
         }
         catch (NumberFormatException e)
         {
            throw new RuntimeException("Make sure all fields ar doubles in " + file.getAbsolutePath(), e);
         }
      }
      else
      {
         System.out.println("File not found or invalid. Default offsets are zero.");
         for (keys key : keys.values())
         {
            deadbands.put(key, 0.0);
         }
      }
   }

   public static double getXDeadband()
   {
      return deadbands.get(keys.DEADBAND_X);
   }

   public static double getYDeadband()
   {
      return deadbands.get(keys.DEADBAND_Y);
   }

   public static double getZDeadband()
   {
      return deadbands.get(keys.DEADBAND_Z);
   }

   public static double getYawDeadbandInDegrees()
   {
      return deadbands.get(keys.YAW_DEADBAND_IN_DEGREES);
   }
   
   public static void main(String[] args)
   {
      System.out.println(ValkyrieLocalizationParameters.getXDeadband());
      System.out.println(ValkyrieLocalizationParameters.getYDeadband());
      System.out.println(ValkyrieLocalizationParameters.getZDeadband());
      System.out.println(ValkyrieLocalizationParameters.getYawDeadbandInDegrees());
      
   }
}
