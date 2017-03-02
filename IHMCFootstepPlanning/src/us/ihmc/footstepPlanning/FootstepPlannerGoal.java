package us.ihmc.footstepPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepPlannerGoal
{
   private FramePose goalPoseBetweenFeet;
   private FramePoint goalPositionBetweenFeet;
   private SimpleFootstep singleFootstepGoal;
   private SideDependentList<SimpleFootstep> doubleFootstepGoal;
   private Point2D xyGoal;
   private double distanceFromXYGoal;

   private FootstepPlannerGoalType footstepPlannerGoalType;

   public FramePose getGoalPoseBetweenFeet()
   {
      return goalPoseBetweenFeet;
   }

   public void setGoalPoseBetweenFeet(FramePose goalPoseBetweenFeet)
   {
      this.goalPoseBetweenFeet = goalPoseBetweenFeet;
   }

   public FramePoint getGoalPositionBetweenFeet()
   {
      return goalPositionBetweenFeet;
   }

   public void setGoalPositionBetweenFeet(FramePoint goalPositionBetweenFeet)
   {
      this.goalPositionBetweenFeet = goalPositionBetweenFeet;
   }

   public SimpleFootstep getSingleFootstepGoal()
   {
      return singleFootstepGoal;
   }

   public void setSingleFootstepGoal(SimpleFootstep singleFootstepGoal)
   {
      this.singleFootstepGoal = singleFootstepGoal;
   }

   public SideDependentList<SimpleFootstep> getDoubleFootstepGoal()
   {
      return doubleFootstepGoal;
   }

   public void setDoubleFootstepGoal(SideDependentList<SimpleFootstep> doubleFootstepGoal)
   {
      this.doubleFootstepGoal = doubleFootstepGoal;
   }

   public void setXYGoal(Point2D xyGoal, double distanceFromXYGoal)
   {
      this.xyGoal = new Point2D(xyGoal);
      this.distanceFromXYGoal = distanceFromXYGoal;
   }

   public Point2D getXYGoal()
   {
      return xyGoal;
   }

   public double getDistanceFromXYGoal()
   {
      return distanceFromXYGoal;
   }

   public FootstepPlannerGoalType getFootstepPlannerGoalType()
   {
      return footstepPlannerGoalType;
   }

   public void setFootstepPlannerGoalType(FootstepPlannerGoalType footstepPlannerGoalType)
   {
      this.footstepPlannerGoalType = footstepPlannerGoalType;
   }

}
