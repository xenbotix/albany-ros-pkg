/*
 * TestGUI.java
 *
 * Created on Apr 14, 2010, 4:56:20 PM
 */

package emotiontest;
import java.awt.Point;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

/**
 *
 * @author Ken
 */

public class TestGUI extends javax.swing.JFrame {
	int x = 50;
	int y = 50;
    /** Creates new form TestGUI */


    public TestGUI() {
		this.setContentPane(new MyPanel());
		initComponents();
		Graphics2D gd = (Graphics2D) getContentPane().getGraphics();
		gd.setColor(Color.RED);
		gd.fillRect(10, 10, 200, 50);
		gd.fillOval(x, y, 10, 10);
		setVisible(true);
    }

    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        s6 = new javax.swing.JLabel();
        s1 = new javax.swing.JLabel();
        s11 = new javax.swing.JLabel();
        s7 = new javax.swing.JLabel();
        s4 = new javax.swing.JLabel();
        s2 = new javax.swing.JLabel();
        s12 = new javax.swing.JLabel();
        s9 = new javax.swing.JLabel();
        s10 = new javax.swing.JLabel();
        s5 = new javax.swing.JLabel();
        s3 = new javax.swing.JLabel();
        s8 = new javax.swing.JLabel();
        s0 = new javax.swing.JLabel();
        coordbox = new javax.swing.JTextField();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setResizable(false);
        addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                formMouseReleased(evt);
            }
        });

        s6.setText("Neutral");

        s1.setText("Overstimulated");

        s11.setText("Understimulated");

        s7.setText("Informed");

        s4.setText("Curious");

        s2.setText("Helpful");

        s12.setText("Content");

        s9.setText("Patient");

        s10.setText("Useless");

        s5.setText("Confused");

        s3.setText("Impatient");

        s8.setText("Apathetic");

        s0.setText("Frustrated");

        coordbox.setEditable(false);
        coordbox.setHorizontalAlignment(javax.swing.JTextField.CENTER);
        coordbox.setText("X: 50, Y: 50");
        coordbox.setFocusable(false);

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING, false)
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(s0)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                .addComponent(s1))
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(s10)
                                .addGap(236, 236, 236)
                                .addComponent(s11)))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 69, Short.MAX_VALUE)
                        .addComponent(coordbox, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(77, 77, 77)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                            .addComponent(s2)
                            .addComponent(s12))
                        .addContainerGap())
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                        .addGap(213, 213, 213)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(s8)
                            .addComponent(s3))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 197, Short.MAX_VALUE)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(s4)
                            .addComponent(s9))
                        .addGap(183, 183, 183))))
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                .addGap(86, 86, 86)
                .addComponent(s5)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 426, Short.MAX_VALUE)
                .addComponent(s7)
                .addGap(85, 85, 85))
            .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                .addGroup(layout.createSequentialGroup()
                    .addGap(0, 333, Short.MAX_VALUE)
                    .addComponent(s6)
                    .addGap(0, 334, Short.MAX_VALUE)))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addContainerGap()
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(s1)
                            .addComponent(s2)
                            .addComponent(s0))
                        .addGap(186, 186, 186)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(s7)
                            .addComponent(s5)))
                    .addGroup(layout.createSequentialGroup()
                        .addGap(87, 87, 87)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(s4)
                            .addComponent(s3))))
                .addGap(88, 88, 88)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(s8)
                    .addComponent(s9))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 78, Short.MAX_VALUE)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                        .addComponent(s11)
                        .addComponent(s12)
                        .addComponent(s10))
                    .addComponent(coordbox, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addContainerGap())
            .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                .addGroup(layout.createSequentialGroup()
                    .addGap(0, 216, Short.MAX_VALUE)
                    .addComponent(s6)
                    .addGap(0, 217, Short.MAX_VALUE)))
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

	private void refresh(){
		coordbox.setText("X: " + x + ", Y: " + y);
		//send x and (100-y) out
		System.out.println("x - " + x + " y - " + (100 - y));
	}

	private void shellCommand(String cmd){
		Runtime run = Runtime.getRuntime();
		try{
			Process pr = run.exec(cmd);
			pr.waitFor();
		} catch (Exception e){
			e.printStackTrace();
		}
	}

	private void formMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_formMouseReleased
		Point p = evt.getPoint();
		int xx = (int) p.getX();
		int yy = (int) p.getY();
		System.out.println("X: " + xx + " and Y: " + yy);
		double temp = (double) xx * 100 / 710;
		x = (int) temp;
		temp = yy * 100 / 440 - 5;
		y = (int) temp;
		if (x > 100) x = 100; if (x < 0) x = 0;
		if (y > 100) y = 100; if (y < 0) y = 0;
		evt.consume();
		refresh();
		String cmnd = "./t " + x + " " + y;
		shellCommand(cmnd);
	}//GEN-LAST:event_formMouseReleased

    /**
    * @param args the command line arguments
    */
    public static void main(String args[]) {
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                new TestGUI().setVisible(true);
            }
        });
    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JTextField coordbox;
    private javax.swing.JLabel s0;
    private javax.swing.JLabel s1;
    private javax.swing.JLabel s10;
    private javax.swing.JLabel s11;
    private javax.swing.JLabel s12;
    private javax.swing.JLabel s2;
    private javax.swing.JLabel s3;
    private javax.swing.JLabel s4;
    private javax.swing.JLabel s5;
    private javax.swing.JLabel s6;
    private javax.swing.JLabel s7;
    private javax.swing.JLabel s8;
    private javax.swing.JLabel s9;
    // End of variables declaration//GEN-END:variables

}