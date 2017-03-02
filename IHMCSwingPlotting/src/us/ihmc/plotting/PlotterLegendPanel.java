package us.ihmc.plotting;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;

import javax.swing.JPanel;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.plotting.artifact.ArtifactsChangedListener;

public class PlotterLegendPanel extends JPanel implements ArtifactsChangedListener
{
   private static final long serialVersionUID = -8268027977270506164L;
   private ArrayList<Artifact> artifacts = new ArrayList<Artifact>();
   private final Plotter2DAdapter graphics2dAdapter;
   
   private final Point2D drawOrigin = new Point2D();

   public PlotterLegendPanel(Plotter2DAdapter graphics2dAdapter)
   {
      super.setBackground(new Color(180, 220, 240));
      
      this.graphics2dAdapter = graphics2dAdapter;
   }

   public void setArtifacts(ArrayList<Artifact> artifacts)
   {
      this.artifacts.clear();
      this.artifacts.addAll(artifacts);
      repaint();
   }

   @Override
   public void paintComponent(Graphics g)
   {
      graphics2dAdapter.setGraphics2d((Graphics2D) g);
      
      super.paintComponent(g);

      int deltaY = 30;

      int artifactX = 30;
      int labelX = 200;
      int y = 20;

      Font f = new Font("Arial", Font.BOLD, 20);
      g.setFont(f);
      g.drawString("Plotter Legend", 130, 20);

      f = new Font("Arial", Font.PLAIN, 14);

      for (Artifact artifact : artifacts)
      {
         y = y + deltaY;
         g.setFont(f);

         drawOrigin.set((double) artifactX, (double) y);
         
         artifact.drawLegend(graphics2dAdapter, drawOrigin);

         g.setColor(Color.black);
//         String newName = "";
//         newName = artifact.getID().substring(0, 1).toUpperCase() + artifact.getID().substring(1, artifact.getID().length());
//
//         for (int i = 1; i < newName.length(); i++)
//         {
//            int ascii = (int) newName.charAt(i);
//            if ((ascii >= 65) && (ascii <= 90))
//            {
//               if (!newName.substring(i - 1, i).equals(" "))
//               {
//                  newName = newName.substring(0, i) + " " + newName.substring(i, newName.length());
//                  i++;
//               }
//            }
//         }

         g.drawString(artifact.getID(), labelX, y);
         setPreferredSize(new Dimension(400, y+deltaY));
      }
   }

   @Override
   public void artifactsChanged(ArrayList<Artifact> newArtifacts)
   {
      setArtifacts(newArtifacts);
      repaint();
   }
}
