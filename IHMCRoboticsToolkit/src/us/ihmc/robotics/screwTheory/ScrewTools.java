package us.ihmc.robotics.screwTheory;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ScrewTools
{
   public static RevoluteJoint addRevoluteJoint(String jointName, RigidBody parentBody, Vector3d jointOffset, Vector3d jointAxis)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslationAndIdentityRotation(jointOffset);

      return addRevoluteJoint(jointName, parentBody, transformToParent, jointAxis);
   }

   public static RevoluteJoint addRevoluteJoint(String jointName, RigidBody parentBody, RigidBodyTransform transformToParent, Vector3d jointAxis)
   {
      String beforeJointName = "before" + jointName;

      ReferenceFrame parentFrame;
      if (parentBody.isRootBody())
         parentFrame = parentBody.getBodyFixedFrame();
      else
         parentFrame = parentBody.getParentJoint().getFrameAfterJoint();

      ReferenceFrame frameBeforeJoint = createOffsetFrame(parentFrame, transformToParent, beforeJointName);

      String afterJointName = jointName;
      RevoluteJoint joint = new RevoluteJoint(afterJointName, parentBody, frameBeforeJoint, new FrameVector(frameBeforeJoint, jointAxis));

      return joint;
   }

   public static PassiveRevoluteJoint addPassiveRevoluteJoint(String jointName, RigidBody parentBody, Vector3d jointOffset, Vector3d jointAxis, boolean isPartOfClosedKinematicLoop)
   {
      return addPassiveRevoluteJoint(jointName, parentBody, TransformTools.createTranslationTransform(jointOffset), jointAxis, isPartOfClosedKinematicLoop);
   }

   public static PassiveRevoluteJoint addPassiveRevoluteJoint(String jointName, RigidBody parentBody, RigidBodyTransform transformToParent, Vector3d jointAxis, boolean isPartOfClosedKinematicLoop)
   {
      String beforeJointName = "before" + jointName;

      ReferenceFrame parentFrame;
      if (parentBody.isRootBody())
         parentFrame = parentBody.getBodyFixedFrame();
      else
         parentFrame = parentBody.getParentJoint().getFrameAfterJoint();

      ReferenceFrame frameBeforeJoint = createOffsetFrame(parentFrame, transformToParent, beforeJointName);

      String afterJointName = jointName;

      return new PassiveRevoluteJoint(afterJointName, parentBody, frameBeforeJoint, new FrameVector(frameBeforeJoint, jointAxis), isPartOfClosedKinematicLoop);
   }

   public static PrismaticJoint addPrismaticJoint(String jointName, RigidBody parentBody, Vector3d jointOffset, Vector3d parentJointAxis)
   {
      return addPrismaticJoint(jointName, parentBody, TransformTools.createTranslationTransform(jointOffset), parentJointAxis);
   }

   public static PrismaticJoint addPrismaticJoint(String jointName, RigidBody parentBody, RigidBodyTransform transformToParent, Vector3d jointAxis)
   {
      String beforeJointName = "before" + jointName;

      ReferenceFrame parentFrame;
      if (parentBody.isRootBody())
         parentFrame = parentBody.getBodyFixedFrame();
      else
         parentFrame = parentBody.getParentJoint().getFrameAfterJoint();

      ReferenceFrame frameBeforeJoint = createOffsetFrame(parentFrame, transformToParent, beforeJointName);

      String afterJointName = jointName;
      PrismaticJoint joint = new PrismaticJoint(afterJointName, parentBody, frameBeforeJoint, new FrameVector(frameBeforeJoint, jointAxis));

      return joint;
   }

   public static RigidBody addRigidBody(String name, InverseDynamicsJoint parentJoint, Matrix3d momentOfInertia, double mass, Vector3d centerOfMassOffset)
   {
      String comFrameName = name + "CoM";
      ReferenceFrame comFrame = createOffsetFrame(parentJoint.getFrameAfterJoint(), centerOfMassOffset, comFrameName);
      RigidBodyInertia inertia = new RigidBodyInertia(comFrame, momentOfInertia, mass);
      RigidBody ret = new RigidBody(name, inertia, parentJoint);

      return ret;
   }

   public static RigidBody addRigidBody(String name, InverseDynamicsJoint parentJoint, Matrix3d momentOfInertia, double mass, RigidBodyTransform inertiaPose)
   {
      String comFrameName = name + "CoM";
      ReferenceFrame comFrame = createOffsetFrame(parentJoint.getFrameAfterJoint(), inertiaPose, comFrameName);
      RigidBodyInertia inertia = new RigidBodyInertia(comFrame, momentOfInertia, mass);
      RigidBody ret = new RigidBody(name, inertia, parentJoint);

      return ret;
   }

   private static ReferenceFrame createOffsetFrame(ReferenceFrame parentFrame, Vector3d offset, String frameName)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslationAndIdentityRotation(offset);

      return createOffsetFrame(parentFrame, transformToParent, frameName);
   }

   public static ReferenceFrame createOffsetFrame(ReferenceFrame parentFrame, RigidBodyTransform transformToParent, String frameName)
   {
      ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);

      return beforeJointFrame;
   }

   public static RigidBody[] computeSuccessors(InverseDynamicsJoint... joints)
   {
      RigidBody[] ret = new RigidBody[joints.length];
      for (int i = 0; i < joints.length; i++)
      {
         InverseDynamicsJoint joint = joints[i];
         ret[i] = joint.getSuccessor();
      }
      return ret;
   }

   public static RigidBody[] computeSubtreeSuccessors(InverseDynamicsJoint... joints)
   {
      ArrayList<RigidBody> rigidBodySuccessors = new ArrayList<RigidBody>();
      ArrayList<RigidBody> rigidBodyStack = new ArrayList<RigidBody>();
      for (InverseDynamicsJoint joint : joints)
      {
         rigidBodyStack.add(joint.getPredecessor());
      }
      while (!rigidBodyStack.isEmpty())
      {
         RigidBody currentBody = rigidBodyStack.remove(0);
         List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
         for (InverseDynamicsJoint joint : childrenJoints)
         {
            rigidBodyStack.add(joint.getSuccessor());
            rigidBodySuccessors.add(joint.getSuccessor());
         }
      }
      RigidBody[] ret = new RigidBody[rigidBodySuccessors.size()];

      return rigidBodySuccessors.toArray(ret);
   }

   public static RigidBody[] computeRigidBodiesAfterThisJoint(InverseDynamicsJoint... joints)
   {
      ArrayList<RigidBody> rigidBodySuccessors = new ArrayList<RigidBody>();
      computeRigidBodiesAfterThisJoint(rigidBodySuccessors, joints);
      RigidBody[] ret = new RigidBody[rigidBodySuccessors.size()];

      return rigidBodySuccessors.toArray(ret);
   }

   public static void computeRigidBodiesAfterThisJoint(ArrayList<RigidBody> rigidBodySuccessorsToPack, InverseDynamicsJoint... joints)
   {
      ArrayList<InverseDynamicsJoint> jointStack = new ArrayList<InverseDynamicsJoint>();
      for (InverseDynamicsJoint joint : joints)
      {
         jointStack.add(joint);
      }
      while (!jointStack.isEmpty())
      {
         InverseDynamicsJoint currentJoint = jointStack.remove(0);
         rigidBodySuccessorsToPack.add(currentJoint.getSuccessor());
         RigidBody currentBody = currentJoint.getSuccessor();
         List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
         for (InverseDynamicsJoint joint : childrenJoints)
         {
            jointStack.add(joint);
         }
      }
   }

   public static void computeRigidBodiesFromRootToThisJoint(ArrayList<RigidBody> rigidBodySuccessorsToPack, InverseDynamicsJoint joint)
   {
      RigidBody predecessorBody = joint.getPredecessor();
      if (predecessorBody == null)
         return;
      if (predecessorBody.isRootBody())
         return;

      rigidBodySuccessorsToPack.add(predecessorBody);
      InverseDynamicsJoint parentJoint = predecessorBody.getParentJoint();
      if (parentJoint == null)
         return;

      computeRigidBodiesFromRootToThisJoint(rigidBodySuccessorsToPack, parentJoint);
   }

   public static RigidBody[] computeSubtreeSuccessors(RigidBody... bodies)
   {
      return computeSuccessors(computeSubtreeJoints(bodies));
   }

   public static RigidBody[] computeSubtreeSuccessors(Set<InverseDynamicsJoint> jointsToExclude, RigidBody... bodies)
   {
      ArrayList<RigidBody> rigidBodySuccessors = new ArrayList<RigidBody>();
      ArrayList<RigidBody> rigidBodyStack = new ArrayList<RigidBody>();
      for (RigidBody body : bodies)
      {
         rigidBodyStack.add(body);
      }
      while (!rigidBodyStack.isEmpty())
      {
         RigidBody currentBody = rigidBodyStack.remove(0);
         List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
         for (InverseDynamicsJoint joint : childrenJoints)
         {
            if (!jointsToExclude.contains(joint))
            {
               rigidBodyStack.add(joint.getSuccessor());
               rigidBodySuccessors.add(joint.getSuccessor());
            }
         }
      }

      RigidBody[] ret = new RigidBody[rigidBodySuccessors.size()];

      return rigidBodySuccessors.toArray(ret);
   }

   public static RigidBody[] computeSupportAndSubtreeSuccessors(RigidBody... bodies)
   {
      return computeSuccessors(computeSupportAndSubtreeJoints(bodies));
   }

   public static InverseDynamicsJoint[] computeSupportAndSubtreeJoints(RigidBody... bodies)
   {
      Set<InverseDynamicsJoint> ret = new LinkedHashSet<InverseDynamicsJoint>();
      for (RigidBody body : bodies)
      {
         ret.addAll(Arrays.asList(computeSupportJoints(body)));
         ret.addAll(Arrays.asList(computeSubtreeJoints(body)));
      }
      return ret.toArray(new InverseDynamicsJoint[ret.size()]);
   }

   public static InverseDynamicsJoint[] computeSupportJoints(RigidBody... bodies)
   {
      Set<InverseDynamicsJoint> supportSet = new LinkedHashSet<InverseDynamicsJoint>();
      for (RigidBody rigidBody : bodies)
      {
         RigidBody rootBody = getRootBody(rigidBody);
         InverseDynamicsJoint[] jointPath = createJointPath(rootBody, rigidBody);
         supportSet.addAll(Arrays.asList(jointPath));
      }

      return supportSet.toArray(new InverseDynamicsJoint[supportSet.size()]);
   }

   public static InverseDynamicsJoint[] computeSubtreeJoints(RigidBody... rootBodies)
   {
      return computeSubtreeJoints(Arrays.asList(rootBodies));
   }

   public static InverseDynamicsJoint[] computeSubtreeJoints(List<RigidBody> rootBodies)
   {
      ArrayList<InverseDynamicsJoint> subtree = new ArrayList<InverseDynamicsJoint>();
      ArrayList<RigidBody> rigidBodyStack = new ArrayList<RigidBody>();
      rigidBodyStack.addAll(rootBodies);

      while (!rigidBodyStack.isEmpty())
      {
         RigidBody currentBody = rigidBodyStack.remove(0);
         List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
         for (InverseDynamicsJoint joint : childrenJoints)
         {
            RigidBody successor = joint.getSuccessor();
            rigidBodyStack.add(successor);
            subtree.add(joint);
         }
      }

      InverseDynamicsJoint[] ret = new InverseDynamicsJoint[subtree.size()];
      return subtree.toArray(ret);
   }

   public static RigidBody getRootBody(RigidBody body)
   {
      RigidBody ret = body;
      while (ret.getParentJoint() != null)
      {
         ret = ret.getParentJoint().getPredecessor();
      }
      return ret;
   }

   public static int[] createParentMap(RigidBody[] allRigidBodiesInOrder)
   {
      int[] parentMap = new int[allRigidBodiesInOrder.length];
      List<RigidBody> rigidBodiesInOrderList = Arrays.asList(allRigidBodiesInOrder); // this doesn't have to be fast
      for (int i = 0; i < allRigidBodiesInOrder.length; i++)
      {
         RigidBody currentBody = allRigidBodiesInOrder[i];
         if (currentBody.isRootBody())
         {
            parentMap[i] = -1;
         }
         else
         {
            RigidBody parentBody = currentBody.getParentJoint().getPredecessor();
            parentMap[i] = rigidBodiesInOrderList.indexOf(parentBody);
         }
      }

      return parentMap;
   }

   public static DenseMatrix64F getTauMatrix(InverseDynamicsJoint[] jointsInOrder)
   {
      int size = 0;
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         size += joint.getDegreesOfFreedom();
      }

      DenseMatrix64F tempMatrix = new DenseMatrix64F(InverseDynamicsJoint.maxDoF, 1);
      DenseMatrix64F ret = new DenseMatrix64F(size, 1);
      int startIndex = 0;
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         int endIndex = startIndex + joint.getDegreesOfFreedom() - 1;
         joint.getTauMatrix(tempMatrix);

         MatrixTools.setMatrixBlock(ret, startIndex, 0, tempMatrix, 0, 0, joint.getDegreesOfFreedom(), 1, 1.0);

         startIndex = endIndex + 1;
      }

      return ret;
   }

   public static OneDoFJoint[] createOneDoFJointPath(RigidBody start, RigidBody end)
   {
      return filterJoints(createJointPath(start, end), OneDoFJoint.class);
   }

   public static InverseDynamicsJoint[] createJointPath(RigidBody start, RigidBody end)
   {
      boolean flip = false;
      RigidBody descendant = start;
      RigidBody ancestor = end;
      int pathLength = computeDistanceToAncestor(descendant, ancestor);
      if (pathLength < 0)
      {
         flip = true;
         descendant = end;
         ancestor = start;
         pathLength = computeDistanceToAncestor(end, start);
      }

      InverseDynamicsJoint[] ret = new InverseDynamicsJoint[pathLength];
      RigidBody currentBody = descendant;
      int i = 0;
      while (currentBody != ancestor)
      {
         int j = flip ? pathLength - 1 - i : i;
         InverseDynamicsJoint parentJoint = currentBody.getParentJoint();
         ret[j] = parentJoint;
         currentBody = parentJoint.getPredecessor();
         i++;
      }

      return ret;
   }

   /**
    * Compute and pack the joint path between two RigidBody in the jointPathToPack.
    * Use the method {@link #computeDistanceToAncestor(RigidBody, RigidBody)} to get the size of the Array to provide.
    * @param jointPathToPack
    * @param start
    * @param end
    * @return the length of the joint path, returns -1 if the the given jointPathToPack is too small.
    */
   public static int createJointPath(InverseDynamicsJoint[] jointPathToPack, RigidBody start, RigidBody end)
   {
      boolean flip = false;
      RigidBody descendant = start;
      RigidBody ancestor = end;
      int pathLength = computeDistanceToAncestor(descendant, ancestor);
      if (pathLength < 0)
      {
         flip = true;
         descendant = end;
         ancestor = start;
         pathLength = computeDistanceToAncestor(end, start);
      }

      if (jointPathToPack == null || jointPathToPack.length < pathLength)
         return -1;

      RigidBody currentBody = descendant;
      int i = 0;
      while (currentBody != ancestor)
      {
         int j = flip ? pathLength - 1 - i : i;
         InverseDynamicsJoint parentJoint = currentBody.getParentJoint();
         jointPathToPack[j] = parentJoint;
         currentBody = parentJoint.getPredecessor();
         i++;
      }

      for (int k = pathLength; k < jointPathToPack.length; k++)
         jointPathToPack[k] = null;

      return pathLength;
   }

   public static OneDoFJoint[] cloneOneDoFJointPath(RigidBody start, RigidBody end)
   {
      return cloneJointPathAndFilter(createOneDoFJointPath(start, end), OneDoFJoint.class);
   }

   public static OneDoFJoint[] cloneOneDoFJointPath(OneDoFJoint[] oneDoFJoints)
   {
      return cloneJointPathAndFilter(oneDoFJoints, OneDoFJoint.class);
   }

   public static <T extends InverseDynamicsJoint> T[] cloneJointPathAndFilter(T[] joints, Class<T> clazz)
   {
      return filterJoints(cloneJointPath(joints), clazz);
   }

   public static <T extends InverseDynamicsJoint> T[] cloneJointPathAndFilter(T[] joints, Class<T> clazz, String suffix)
   {
      return filterJoints(cloneJointPath(joints, suffix), clazz);
   }

   public static InverseDynamicsJoint[] cloneJointPath(InverseDynamicsJoint[] inverseDynamicsJoints)
   {
      String clonedJointNameSuffix = "Copy";

      return cloneJointPath(inverseDynamicsJoints, clonedJointNameSuffix);
   }

   public static InverseDynamicsJoint[] cloneJointPath(InverseDynamicsJoint[] inverseDynamicsJoints, String suffix)
   {
      InverseDynamicsJoint[] cloned = new InverseDynamicsJoint[inverseDynamicsJoints.length];

      for (int i = 0; i < inverseDynamicsJoints.length; i++)
      {
         if (inverseDynamicsJoints[i] instanceof RevoluteJoint)
         {
            RevoluteJoint jointOriginal = (RevoluteJoint) inverseDynamicsJoints[i];

            RigidBody predecessorOriginal = jointOriginal.getPredecessor();
            RigidBody predecessorCopy;

            if (i > 0)
            {
               predecessorCopy = cloned[i - 1].getSuccessor();
            }
            else
            {
               String predecessorNameOriginal = predecessorOriginal.getName();
               ReferenceFrame predecessorFrameAfterParentJointOriginal = predecessorOriginal.getParentJoint().getFrameAfterJoint();
               predecessorCopy = new RigidBody(predecessorNameOriginal + suffix, predecessorFrameAfterParentJointOriginal);
            }

            String jointNameOriginal = jointOriginal.getName();
            RigidBodyTransform jointTransform = jointOriginal.getOffsetTransform3D();
            Vector3d jointAxisCopy = jointOriginal.getJointAxis().getVectorCopy();
            RevoluteJoint jointCopy = ScrewTools.addRevoluteJoint(jointNameOriginal + suffix, predecessorCopy, jointTransform, jointAxisCopy);
            jointCopy.setJointLimitLower(jointOriginal.getJointLimitLower());
            jointCopy.setJointLimitUpper(jointOriginal.getJointLimitUpper());
            cloned[i] = jointCopy;
         }
         else if (inverseDynamicsJoints[i] instanceof SixDoFJoint)
         {
            SixDoFJoint jointOriginal = (SixDoFJoint) inverseDynamicsJoints[i];
            RigidBody rootBody = jointOriginal.getPredecessor();

            if (rootBody.getParentJoint() != null)
               throw new RuntimeException("The SixDoFJoint predecessor is not the root body. Case not handled.");

            String rootBodyNameOriginal = rootBody.getName();
            ReferenceFrame rootBodyFrame = rootBody.getBodyFixedFrame();
            RigidBody rootBodyCopy = new RigidBody(rootBodyNameOriginal + suffix, rootBodyFrame);

            String jointNameOriginal = jointOriginal.getName();
            SixDoFJoint jointCopy = new SixDoFJoint(jointNameOriginal + suffix, rootBodyCopy, rootBodyFrame);
            cloned[i] = jointCopy;
         }
         else
         {
            throw new RuntimeException("Not implemented for joints of the type: " + inverseDynamicsJoints[i].getClass().getSimpleName());
         }

         FramePoint comOffset = new FramePoint();
         RigidBody successorOriginal = inverseDynamicsJoints[i].getSuccessor();
         successorOriginal.getCoMOffset(comOffset);
         comOffset.changeFrame(inverseDynamicsJoints[i].getFrameAfterJoint());
         String successorNameOriginal = successorOriginal.getName();
         Matrix3d successorMassMomentOfInertiaPartCopy = successorOriginal.getInertia().getMassMomentOfInertiaPartCopy();
         double successorMass = successorOriginal.getInertia().getMass();
         Vector3d successorCoMOffsetCopy = comOffset.getVectorCopy();
         RigidBody successorCopy = ScrewTools.addRigidBody(successorNameOriginal + suffix, cloned[i], successorMassMomentOfInertiaPartCopy, successorMass, successorCoMOffsetCopy);
         cloned[i].setSuccessor(successorCopy);
      }
      return cloned;
   }

   public static <T extends InverseDynamicsJoint> T[] cloneJointPathDisconnectedFromOriginalRobot(T[] joints, Class<T> clazz, String suffix, ReferenceFrame rootBodyFrame)
   {
      return filterJoints(cloneJointPathDisconnectedFromOriginalRobot(joints, suffix, rootBodyFrame), clazz);
   }

   public static InverseDynamicsJoint[] cloneJointPathDisconnectedFromOriginalRobot(InverseDynamicsJoint[] inverseDynamicsJoints, String suffix, ReferenceFrame rootBodyFrame)
   {
      InverseDynamicsJoint[] cloned = new InverseDynamicsJoint[inverseDynamicsJoints.length];

      for (int i = 0; i < inverseDynamicsJoints.length; i++)
      {
         if (inverseDynamicsJoints[i] instanceof RevoluteJoint)
         {
            RevoluteJoint jointOriginal = (RevoluteJoint) inverseDynamicsJoints[i];

            RigidBody predecessorOriginal = jointOriginal.getPredecessor();
            RigidBody predecessorCopy;

            if (i > 0)
            {
               predecessorCopy = cloned[i - 1].getSuccessor();
            }
            else
            {
               String predecessorNameOriginal = predecessorOriginal.getName();
               predecessorCopy = new RigidBody(predecessorNameOriginal + suffix, rootBodyFrame);
            }

            String jointNameOriginal = jointOriginal.getName();
            RigidBodyTransform jointTransform = jointOriginal.getOffsetTransform3D();
            Vector3d jointAxisCopy = jointOriginal.getJointAxis().getVectorCopy();
            RevoluteJoint jointCopy = ScrewTools.addRevoluteJoint(jointNameOriginal + suffix, predecessorCopy, jointTransform, jointAxisCopy);
            jointCopy.setJointLimitLower(jointOriginal.getJointLimitLower());
            jointCopy.setJointLimitUpper(jointOriginal.getJointLimitUpper());
            cloned[i] = jointCopy;
         }
         else
         {
            throw new RuntimeException("Not implemented for joints of the type: " + inverseDynamicsJoints[i].getClass().getSimpleName());
         }

         RigidBody successorOriginal = inverseDynamicsJoints[i].getSuccessor();
         FramePoint comOffset = new FramePoint();
         successorOriginal.getCoMOffset(comOffset);
         comOffset.changeFrame(inverseDynamicsJoints[i].getFrameAfterJoint());
         String successorNameOriginal = successorOriginal.getName();
         Matrix3d successorMassMomentOfInertiaPartCopy = successorOriginal.getInertia().getMassMomentOfInertiaPartCopy();
         double successorMass = successorOriginal.getInertia().getMass();
         Vector3d successorCoMOffsetCopy = comOffset.getVectorCopy();
         RigidBody successorCopy = ScrewTools.addRigidBody(successorNameOriginal + suffix, cloned[i], successorMassMomentOfInertiaPartCopy, successorMass, successorCoMOffsetCopy);
         cloned[i].setSuccessor(successorCopy);
      }
      return cloned;
   }

   /**
    * Traverses up the kinematic chain from the candidate descendant towards the root body, checking to see if each parent body is the ancestor in question.
    * @param candidateDescendant
    * @param ancestor
    * @return
    */
   public static boolean isAncestor(RigidBody candidateDescendant, RigidBody ancestor)
   {
      RigidBody currentBody = candidateDescendant;
      while (!currentBody.isRootBody())
      {
         if (currentBody == ancestor)
         {
            return true;
         }
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      return currentBody == ancestor;
   }

   public static int computeDistanceToAncestor(RigidBody descendant, RigidBody ancestor)
   {
      int ret = 0;
      RigidBody currentBody = descendant;
      while (!currentBody.isRootBody() && (currentBody != ancestor))
      {
         ret++;
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      if (currentBody != ancestor)
         ret = -1;

      return ret;
   }

   public static void getJointVelocitiesMatrix(InverseDynamicsJoint[] joints, DenseMatrix64F jointVelocitiesMatrixToPack)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         int dof = joint.getDegreesOfFreedom();
         joint.getVelocityMatrix(jointVelocitiesMatrixToPack, rowStart);
         rowStart += dof;
      }
   }

   public static void getJointVelocitiesMatrix(Iterable<? extends InverseDynamicsJoint> joints, DenseMatrix64F jointVelocitiesMatrixToPack)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         int dof = joint.getDegreesOfFreedom();
         joint.getVelocityMatrix(jointVelocitiesMatrixToPack, rowStart);
         rowStart += dof;
      }
   }

   public static void getDesiredJointAccelerationsMatrix(InverseDynamicsJoint[] joints, DenseMatrix64F desiredJointAccelerationsMatrixToPack)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         int dof = joint.getDegreesOfFreedom();
         joint.getDesiredAccelerationMatrix(desiredJointAccelerationsMatrixToPack, rowStart);
         rowStart += dof;
      }
   }

   public static int computeDegreesOfFreedom(InverseDynamicsJoint[] jointList)
   {
      int ret = 0;
      for (InverseDynamicsJoint joint : jointList)
      {
         ret += joint.getDegreesOfFreedom();
      }

      return ret;
   }

   public static int computeDegreesOfFreedom(Iterable<? extends InverseDynamicsJoint> jointList)
   {
      int ret = 0;
      for (InverseDynamicsJoint joint : jointList)
      {
         ret += joint.getDegreesOfFreedom();
      }

      return ret;
   }

   public static int computeDegreesOfFreedom(List<? extends InverseDynamicsJoint> jointList)
   {
      int ret = 0;
      for (int i = 0; i < jointList.size(); i++)
      {
         ret += jointList.get(i).getDegreesOfFreedom();
      }
      return ret;
   }

   public static SpatialAccelerationVector createGravitationalSpatialAcceleration(RigidBody rootBody, double gravity)
   {
      Vector3d gravitationalAcceleration = new Vector3d(0.0, 0.0, gravity);
      Vector3d zero = new Vector3d();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), rootBody.getBodyFixedFrame(), gravitationalAcceleration, zero);

      return rootAcceleration;
   }

   public static void getJointPositions(InverseDynamicsJoint[] joints, DenseMatrix64F jointPositionsToPack)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         joint.getConfigurationMatrix(jointPositionsToPack, rowStart);
         rowStart += joint.getDegreesOfFreedom();
         if (joint instanceof SixDoFJoint || joint instanceof SphericalJoint)
            rowStart++; // Because of stupid quaternions
      }
   }

   public static void getJointDesiredPositions(OneDoFJoint[] joints, DenseMatrix64F jointPositionsToPack)
   {
      int rowStart = 0;
      for (OneDoFJoint joint : joints)
      {
         jointPositionsToPack.set(rowStart, 0, joint.getqDesired());
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setJointPositions(InverseDynamicsJoint[] joints, DenseMatrix64F jointPositions)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         joint.setConfiguration(jointPositions, rowStart);
         rowStart += joint.getDegreesOfFreedom();
         if (joint instanceof SixDoFJoint || joint instanceof SphericalJoint)
            rowStart++; // Because of stupid quaternions
      }
   }

   public static void setDesiredJointPositions(OneDoFJoint[] joints, DenseMatrix64F jointPositions)
   {
      int rowStart = 0;
      for (OneDoFJoint joint : joints)
      {
         joint.setqDesired(jointPositions.get(rowStart, 0));
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setDesiredJointVelocities(OneDoFJoint[] joints, DenseMatrix64F jointVelocities)
   {
      int rowStart = 0;
      for (OneDoFJoint joint : joints)
      {
         joint.setQdDesired(jointVelocities.get(rowStart, 0));
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setDesiredAccelerations(InverseDynamicsJoint[] jointList, DenseMatrix64F jointAccelerations)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : jointList)
      {
         joint.setDesiredAcceleration(jointAccelerations, rowStart);
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setVelocities(InverseDynamicsJoint[] jointList, DenseMatrix64F jointVelocities)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : jointList)
      {
         joint.setVelocity(jointVelocities, rowStart);
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setJointAccelerations(OneDoFJoint[] jointList, DenseMatrix64F jointAccelerations)
   {
      int rowStart = 0;
      for (OneDoFJoint joint : jointList)
      {
         joint.setQdd(jointAccelerations.get(rowStart, 0));
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void computeIndicesForJoint(InverseDynamicsJoint[] jointsInOrder, TIntArrayList listToPackIndices, InverseDynamicsJoint... jointsToComputeIndicesFor)
   {
      int startIndex = 0;
      for (int i = 0; i < jointsInOrder.length; i++)
      {
         int nDegreesOfFreedom = jointsInOrder[i].getDegreesOfFreedom();

         for (int j = 0; j < jointsToComputeIndicesFor.length; j++)
         {
            if (jointsInOrder[i] == jointsToComputeIndicesFor[j])
            {
               for (int k = startIndex; k < startIndex + nDegreesOfFreedom; k++)
               {
                  listToPackIndices.add(k);
               }
            }
         }

         startIndex += nDegreesOfFreedom;
      }
   }

   public static void computeIndexForJoint(InverseDynamicsJoint[] jointsInOrder, TIntArrayList listToPackIndices, InverseDynamicsJoint jointToComputeIndicesFor)
   {
      int startIndex = 0;
      for (int i = 0; i < jointsInOrder.length; i++)
      {
         int nDegreesOfFreedom = jointsInOrder[i].getDegreesOfFreedom();

         if (jointsInOrder[i] == jointToComputeIndicesFor)
         {
            for (int k = startIndex; k < startIndex + nDegreesOfFreedom; k++)
            {
               listToPackIndices.add(k);
            }
         }

         startIndex += nDegreesOfFreedom;
      }
   }

   public static RevoluteJoint[] extractRevoluteJoints(InverseDynamicsJoint[] allJoints)
   {
      if (allJoints == null)
         return null;

      ArrayList<RevoluteJoint> revoluteJointsList = new ArrayList<RevoluteJoint>();
      for (InverseDynamicsJoint joint : allJoints)
      {
         if (joint instanceof RevoluteJoint)
            revoluteJointsList.add((RevoluteJoint) joint);
      }

      RevoluteJoint[] revoluteJointArray = new RevoluteJoint[revoluteJointsList.size()];
      revoluteJointsList.toArray(revoluteJointArray);

      return revoluteJointArray;
   }

   public static <T extends InverseDynamicsJoint> int computeNumberOfJointsOfType(Class<T> clazz, InverseDynamicsJoint[] joints)
   {
      int ret = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         if (clazz.isAssignableFrom(joint.getClass()))
            ret++;
      }

      return ret;
   }

   public static <T extends InverseDynamicsJoint> T[] filterJoints(InverseDynamicsJoint[] source, Class<T> clazz)
   {
      @SuppressWarnings("unchecked")
      T[] retArray = (T[]) Array.newInstance(clazz, ScrewTools.computeNumberOfJointsOfType(clazz, source));
      filterJoints(source, retArray, clazz);
      return retArray;
   }

   public static <T extends InverseDynamicsJoint> void filterJoints(InverseDynamicsJoint[] source, Object[] dest, Class<T> clazz)
   {
      int index = 0;
      for (InverseDynamicsJoint joint : source)
      {
         if (clazz.isAssignableFrom(joint.getClass()))
         {
            dest[index++] = joint;
         }
      }
   }

   public static InverseDynamicsJoint[] findJointsWithNames(InverseDynamicsJoint[] allJoints, String... jointNames)
   {
      if (jointNames == null)
         return null;

      InverseDynamicsJoint[] ret = new InverseDynamicsJoint[jointNames.length];
      int index = 0;
      for (InverseDynamicsJoint joint : allJoints)
      {
         for (String jointName : jointNames)
         {
            if (joint.getName().equals(jointName))
               ret[index++] = joint;
         }
      }

      if (index != jointNames.length)
         throw new RuntimeException("Not all joints could be found");

      return ret;
   }

   public static RigidBody[] findRigidBodiesWithNames(RigidBody[] allBodies, String... names)
   {
      RigidBody[] ret = new RigidBody[names.length];
      int index = 0;
      for (RigidBody body : allBodies)
      {
         for (String name : names)
         {
            if (body.getName().equals(name))
               ret[index++] = body;
         }
      }

      if (index != names.length)
         throw new RuntimeException("Not all bodies could be found");

      return ret;
   }

   public static void addExternalWrenches(Map<RigidBody, Wrench> externalWrenches, Map<RigidBody, Wrench> wrenchMapToAdd)
   {
      for (RigidBody rigidBody : wrenchMapToAdd.keySet())
      {
         Wrench externalWrenchToCompensateFor = wrenchMapToAdd.get(rigidBody);

         Wrench externalWrench = externalWrenches.get(rigidBody);
         if (externalWrench == null)
         {
            externalWrenches.put(rigidBody, new Wrench(externalWrenchToCompensateFor));
         }
         else
         {
            externalWrench.add(externalWrenchToCompensateFor);
         }
      }
   }

   public static long computeGeometricJacobianNameBasedHashCode(InverseDynamicsJoint joints[], ReferenceFrame jacobianFrame, boolean allowChangeFrame)
   {
      long jointsHashCode = NameBasedHashCodeTools.computeArrayHashCode(joints);
      if (!allowChangeFrame)
         return NameBasedHashCodeTools.combineHashCodes(jointsHashCode, jacobianFrame);
      else
         return jointsHashCode;
   }

   public static long computeGeometricJacobianNameBasedHashCode(InverseDynamicsJoint joints[], int firstIndex, int lastIndex, ReferenceFrame jacobianFrame, boolean allowChangeFrame)
   {
      long jointsHashCode = NameBasedHashCodeTools.computeSubArrayHashCode(joints, firstIndex, lastIndex);
      if (!allowChangeFrame)
         return NameBasedHashCodeTools.combineHashCodes(jointsHashCode, jacobianFrame);
      else
         return jointsHashCode;
   }
}