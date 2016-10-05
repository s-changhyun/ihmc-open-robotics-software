package us.ihmc.robotbuilder.util;

import javaslang.collection.List;
import org.junit.Test;
import us.ihmc.robotbuilder.util.TreeFocus.Breadcrumb;
import us.ihmc.tools.testing.JUnitTools;

import java.util.Optional;

import static javaslang.collection.List.empty;
import static org.junit.Assert.*;

/**
 *
 */
public class TreeFocusTest
{
   private static final Tree<Integer> BINARY_TREE = new Tree<>(0, List.of(new Tree<>(1, empty()), new Tree<>(2, empty())));
   private static final Tree<Integer> SINGLETON_TREE = new Tree<>(0, empty());

   @Test public void testEqualsHashCode()
   {
      //noinspection OptionalGetWithoutIsPresent
      JUnitTools.testHashCodeEqualsMethods(BINARY_TREE.getFocus(), BINARY_TREE.getFocus(), BINARY_TREE.getFocus().firstChild().get());
      JUnitTools.testHashCodeEqualsMethods(new Breadcrumb<>(BINARY_TREE, empty(), empty()),
                                           new Breadcrumb<>(BINARY_TREE, empty(), empty()),
                                           new Breadcrumb<>(SINGLETON_TREE, empty(), empty()));
   }

   @Test public void testToStringContainsUsefulInformation()
   {
      assertTrue(new Tree<>("ABCD1234", empty()).getFocus().toString().contains("ABCD1234"));
      String childFocusString = BINARY_TREE.getFocus().firstChild().map(TreeFocus::toString).orElse("");
      assertTrue(childFocusString.contains("1") && childFocusString.contains("0") && childFocusString.contains("2")); // both root and siblings
   }

   @Test public void testRootOfRootFocusReturnsTheSameInstance()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      assertTrue(root == root.root());
   }

   @Test public void testRootHasNoParent()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      assertFalse(root.parent().isPresent());
   }

   @Test public void testSingletonHasNoChildren()
   {
      assertFalse(SINGLETON_TREE.getFocus().firstChild().isPresent());
   }

   @Test public void testFirstChildReturnsTheCorrectChild()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> firstChild = root.firstChild();
      assertTrue(firstChild.isPresent());
      assertEquals(1, (int) firstChild.get().getFocusedNode().getValue());
   }

   @Test public void testSiblingOfFirstChildIsTheSecondChild()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> firstChild = root.firstChild();
      Optional<TreeFocus<Tree<Integer>>> secondChild = firstChild.flatMap(TreeFocus::nextSibling);
      assertTrue(secondChild.isPresent());
      assertEquals(2, (int) secondChild.get().getFocusedNode().getValue());
   }

   @Test public void testSiblingOfSecondChildIsTheFirstChild()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> secondChild = root.firstChild().flatMap(TreeFocus::nextSibling);
      Optional<TreeFocus<Tree<Integer>>> firstChild = secondChild.flatMap(TreeFocus::previousSibling);
      assertTrue(secondChild.isPresent());
      assertEquals(root.firstChild(), firstChild);
   }

   @Test public void testRootFindsTreeRootFromChild()
   {
      Optional<TreeFocus<Tree<Integer>>> firstChild = BINARY_TREE.getFocus().firstChild();
      assertTrue(firstChild.isPresent());
      assertEquals(BINARY_TREE.getFocus(), firstChild.get().root());
   }

   @Test public void testRootHasNoSiblings()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      assertFalse(root.nextSibling().isPresent());
      assertFalse(root.previousSibling().isPresent());
   }

   @Test public void testFindLocatesChildNode()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> found1 = root.findChild(node -> node.getValue() == 1);
      assertTrue(found1.isPresent() && found1.get().getFocusedNode().getValue() == 1);

      Optional<TreeFocus<Tree<Integer>>> found2 = root.findChild(node -> node.getValue() == 2);
      assertTrue(found2.isPresent() && found2.get().getFocusedNode().getValue() == 2);
   }

   @Test public void testReplaceCreatesANewTreeWithTheReplacedNode()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      root.firstChild().map(child ->
                            {
                               TreeFocus<Tree<Integer>> replaced = child.replace(new Tree<>(3));
                               TreeFocus<Tree<Integer>> replacedRoot = replaced.root();
                               assertNotEquals(replacedRoot.getFocusedNode(), root);
                               assertEquals(3, (int)replacedRoot.getFocusedNode().getChild(0).getValue());
                               return child;
                            })
      .orElseGet(() -> {
         assertTrue("Root has no children", false);
         return null;
      });
   }

   @Test public void testAddSiblingDoesNothingOnRootNode()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      assertFalse(root.addLeftSibling(new Tree<>(1)).isPresent());
      assertFalse(root.addRightSibling(new Tree<>(1)).isPresent());
   }

   @Test public void testAddLeftSiblingAtTheBeginning()
   {
      BINARY_TREE.getFocus().firstChild()
            .flatMap(firstChild -> firstChild.addLeftSibling(new Tree<>(3)))
            .map(addedSibling -> {
               assertEquals(3, (int)addedSibling.getFocusedNode().getValue());
               assertNotEquals(addedSibling.root(), BINARY_TREE.getFocus());

               assertEquals(BINARY_TREE.firstChild(), addedSibling.nextSibling().map(TreeFocus::getFocusedNode));

               assertFalse(addedSibling.previousSibling().isPresent());
               return addedSibling;
            })
      .orElseGet(() -> {
         assertTrue("Root has no children", false);
         return null;
      });
   }

   @Test public void testAddLeftSiblingAtTheEnd()
   {
      BINARY_TREE.getFocus().firstChild()
                 .flatMap(TreeFocus::nextSibling)
                 .flatMap(lastChild -> lastChild.addLeftSibling(new Tree<>(3)))
                 .map(addedSibling -> {
                    assertEquals(3, (int)addedSibling.getFocusedNode().getValue());
                    assertNotEquals(addedSibling.root(), BINARY_TREE.getFocus());

                    assertEquals(Optional.of(1), addedSibling.previousSibling().map(TreeFocus::getFocusedNode).map(Tree::getValue));
                    assertEquals(Optional.of(2), addedSibling.nextSibling().map(TreeFocus::getFocusedNode).map(Tree::getValue));
                    return addedSibling;
                 })
                 .orElseGet(() -> {
                    assertTrue("Root has no children", false);
                    return null;
                 });
   }

   @Test public void testAddRightSiblingAtTheBeginning()
   {
      BINARY_TREE.getFocus().firstChild()
                 .flatMap(firstChild -> firstChild.addRightSibling(new Tree<>(3)))
                 .map(addedSibling -> {
                    assertEquals(3, (int)addedSibling.getFocusedNode().getValue());
                    assertNotEquals(addedSibling.root(), BINARY_TREE.getFocus());

                    assertEquals(Optional.of(1), addedSibling.previousSibling().map(TreeFocus::getFocusedNode).map(Tree::getValue));
                    assertEquals(Optional.of(2), addedSibling.nextSibling().map(TreeFocus::getFocusedNode).map(Tree::getValue));
                    return addedSibling;
                 })
                 .orElseGet(() -> {
                    assertTrue("Root has no children", false);
                    return null;
                 });
   }

   @Test public void testAddRightSiblingAtTheEnd()
   {
      BINARY_TREE.getFocus().firstChild()
                 .flatMap(firstChild -> firstChild.addLeftSibling(new Tree<>(3)))
                 .map(addedSibling -> {
                    assertEquals(3, (int)addedSibling.getFocusedNode().getValue());
                    assertNotEquals(addedSibling.root(), BINARY_TREE.getFocus());

                    assertEquals(BINARY_TREE.firstChild(), addedSibling.nextSibling().map(TreeFocus::getFocusedNode));

                    assertFalse(addedSibling.previousSibling().isPresent());
                    return addedSibling;
                 })
                 .orElseGet(() -> {
                    assertTrue("Root has no children", false);
                    return null;
                 });
   }

   @Test public void testAppendChild()
   {
      Tree<Integer> newChild = new Tree<>(3);
      TreeFocus<Tree<Integer>> newChildFocus = BINARY_TREE.getFocus().appendChild(newChild);

      Tree<Integer> newRoot = newChildFocus.root().getFocusedNode();
      assertEquals(3, newRoot.countChildren());

      assertEquals(newRoot.findValue(x -> x == 3)
                          .flatMap(TreeFocus::previousSibling)
                          .map(TreeFocus::getFocusedNode)
                          .map(Tree::getValue),
                   Optional.of(2));
   }

   @Test public void testPrependChild()
   {
      Tree<Integer> newChild = new Tree<>(3);
      TreeFocus<Tree<Integer>> newChildFocus = BINARY_TREE.getFocus().prependChild(newChild);

      Tree<Integer> newRoot = newChildFocus.root().getFocusedNode();
      assertEquals(3, newRoot.childStream().count());

      assertEquals(newRoot.findValue(x -> x == 3)
                          .flatMap(TreeFocus::nextSibling)
                          .map(TreeFocus::getFocusedNode)
                          .map(Tree::getValue),
                   Optional.of(1));
   }

   @Test public void testRemoveRootReturnsEmptyFocus()
   {
      assertFalse(BINARY_TREE.getFocus().remove().isPresent());
   }

   @Test public void testRemoveChildReturnsRootWithoutTheChild()
   {
      BINARY_TREE.getFocus()
                 .firstChild()
                 .flatMap(TreeFocus::remove)
      .map(removedParent -> {
         assertEquals(1, removedParent.getFocusedNode().childStream().count());
         assertEquals(2, (int)removedParent.getFocusedNode().getChild(0).getValue());
         return removedParent;
      }).orElseGet(() -> {
         assertTrue("Removed node should have a parent", false);
         return null;
      });
   }
}