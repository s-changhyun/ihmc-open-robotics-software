package us.ihmc.atlas.calib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.so.Rodrigues_F64;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class CalibUtil
{
   protected static final Quaternion quat0 = new Quaternion(0, 0, 0, 1);
   protected static final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   public static Map<String, Double> addQ(Map<String, Double> q1, Map<String, Double> q2)
   {
      Map<String, Double> qbuffer = new HashMap<>();
      addQ(q1, q2, qbuffer);
      return qbuffer;
   }

   public static void addQ(Map<String, Double> q1, Map<String, Double> q2, Map<String, Double> qout)
   {
      assert (q1.size() == q2.size());
      for (String jointName : q1.keySet())
      {
         //      System.out.println("addQ:"+jointName);
         double v = 0;
         if (q1.containsKey(jointName))
            v += q1.get(jointName);
         if (q2.containsKey(jointName))
            v += q2.get(jointName);
         qout.put(jointName, v);
      }
   }

   public static ArrayList<String> toStringArrayList(ArrayList<OneDoFJoint> joints)
   {
      ArrayList<String> jointNames = new ArrayList<>();
      for (OneDoFJoint joint : joints)
         jointNames.add(joint.getName());
      return jointNames;
   }

   public static void setRobotModelFromData(FullRobotModel fullRobotModel, Map<String, Double> qmap)
   {
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         if (qmap.containsKey(joint.getName()))
         {
            joint.setQ(qmap.get(joint.getName()));
         } else if (AtlasKinematicCalibrator.DEBUG)
         {
            System.out.println("model contain joints not in data " + joint.getName());
            joint.setQ(0);
         }
      }
   }

   public static void setRobotModelFromData(FullRobotModel fullRobotModel, Map<String, Double> qmap, Map<String, Double> qbias)
   {
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         if (qmap.containsKey(joint.getName()))
         {
            double bias = 0;
            if (qbias.containsKey(joint.getName()))
            {
               bias = qbias.get(joint.getName());
            }

            joint.setQ(qmap.get(joint.getName()) + bias);
         } else if (AtlasKinematicCalibrator.DEBUG)
         {
            System.out.println("model contain joints not in data " + joint.getName());
            joint.setQ(0);
         }
      }
   }

   public static Vector3D matrix3dToAxisAngle3d(RotationMatrix m)
   {
      DenseMatrix64F md = new DenseMatrix64F(3, 3);
      m.get(md);
      return matrix3dToAxisAngle3d(md);
   }

   public static Vector3D matrix3dToAxisAngle3d(DenseMatrix64F md)
   {
      Rodrigues_F64 r = ConvertRotation3D_F64.matrixToRodrigues(md, (Rodrigues_F64)null);
      r.unitAxisRotation.scale(r.theta);
      Vector3D angleAxis = new Vector3D(r.unitAxisRotation.x, r.unitAxisRotation.y, r.unitAxisRotation.z);
      return angleAxis;
   }

   public static Vector3D rotationDiff(RotationMatrix r1, RotationMatrix r2)
   {
      DenseMatrix64F m1 = new DenseMatrix64F(3, 3), m2 = new DenseMatrix64F(3, 3);
      r1.get(m1);
      r2.get(m2);

      DenseMatrix64F mDiff = new DenseMatrix64F(3, 3);
      CommonOps.multTransB(m1, m2, mDiff);
      return matrix3dToAxisAngle3d(mDiff);
   }
}