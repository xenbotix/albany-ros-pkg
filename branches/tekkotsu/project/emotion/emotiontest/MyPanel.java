/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * MyPanel.java
 *
 * Created on May 3, 2010, 4:32:53 PM
 */

package emotiontest;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import javax.swing.JPanel;

public class MyPanel extends JPanel
{
	MyPanel(){
		this.setVisible(true);
		this.setOpaque(false);
	}

   @Override public void paintComponent(Graphics g)
   {
      Graphics2D g2d = (Graphics2D)g;
      g2d.setPaint(Color.GREEN);
      g2d.fillOval(352, 219, 10, 10);  //neutral
	  g2d.fillOval(0,0,10,10);  //frustrated
	  g2d.fillOval(0, 438, 10, 10);  //useless
	  g2d.fillOval(705, 438, 10, 10);  //content
	  g2d.fillOval(705, 0, 10, 10);  //helpful
	  g2d.fillOval(352,0,10,10);  //overstimulated
	  g2d.fillOval(352, 438,10,10);  //understimulated
	  g2d.fillOval(209,109,10,10);  //impatient
	  g2d.fillOval(209,327,10,10);  //apathetic
	  g2d.fillOval(70,217,10,10);  //confused
	  g2d.fillOval(495,109,10,10);  //curious
	  g2d.fillOval(495,327,10,10);
	  g2d.fillOval(640,217,10,10);  //informed
   }



/*
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        setOpaque(false);

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 400, Short.MAX_VALUE)
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 300, Short.MAX_VALUE)
        );
    }// </editor-fold>//GEN-END:initComponents

*/
    // Variables declaration - do not modify//GEN-BEGIN:variables
    // End of variables declaration//GEN-END:variables

}