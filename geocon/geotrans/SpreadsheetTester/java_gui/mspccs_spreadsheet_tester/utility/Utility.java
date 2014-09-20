// CLASSIFICATION: UNCLASSIFIED
package mspccs_spreadsheet_tester.utility;



public class Utility
{
    /** 
     * Center a component over its parent.
     */
    public static void center(java.awt.Container parent, java.awt.Component comp) 
    {
        int x, y; 
        java.awt.Rectangle boundingRect; 
        java.awt.Dimension compSize = comp.getSize();
        
        // If Container is null then our bounding rectangle is the whole screen 
        if (parent == null)  
        { 
            boundingRect = new java.awt.Rectangle(comp.getToolkit().getScreenSize()); 
            boundingRect.setLocation(0,0); 
        } 
        // Else bounding rectangle is the Container 
        else 
            boundingRect = parent.getBounds(); 
        
        // Place the component so its center is the same 
        // as the center of the bounding rectangle 
        x = boundingRect.x + ((boundingRect.width/2) - (compSize.width/2)); 
        y = boundingRect.y + ((boundingRect.height/2) - (compSize.height/2)); 
        
        comp.setLocation(x, y);
    } 

    /** 
     * Change title bar icon.
     */
    public static void setIcon(javax.swing.JFrame jFrame, String icon)
    {
        java.net.URL url = jFrame.getClass().getResource(icon);
        try
        {
            java.awt.Image img = jFrame.createImage((java.awt.image.ImageProducer)url.getContent());
            jFrame.setIconImage(img);
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }
    
    /** 
     * Extract a code from a string of the form code: name.
     */
    public static String parseString(String str)
    {
        String result = "";
        java.util.StringTokenizer strTokenizer = new java.util.StringTokenizer(str, ":");
        if (strTokenizer.hasMoreTokens())
            result = strTokenizer.nextToken();

        return result;
    }

    /** 
     * Return the text field value as a double.
     */
    public static double getDoubleTextField(javax.swing.JTextField textField)
    {
        return Double.parseDouble(textField.getText().trim());
    }
    
    /** 
     * Convert a string to double.
     */
    public static double toDouble(String str)
    {
        return Double.parseDouble(str);
    }
    
    /** 
     * Set a text field with the given value using a precision of 10.
     */
    public static void setTextField(javax.swing.JTextField textField, double value)
    {
        int precision = 10;

        java.text.NumberFormat nf = java.text.NumberFormat.getNumberInstance();
        nf.setMaximumFractionDigits(precision);
        nf.setMinimumFractionDigits(precision);
        nf.setGroupingUsed(false);

        textField.setText(nf.format(value));
        textField.setCaretPosition(0);
    }

    /** 
     * Set a text field with the given value using the given precision.
     */
    public static void setTextField(javax.swing.JTextField textField, double value, int precision)
    {
        java.text.NumberFormat nf = java.text.NumberFormat.getNumberInstance();
        nf.setMaximumFractionDigits(precision);
        nf.setMinimumFractionDigits(precision);
        nf.setGroupingUsed(false);

        textField.setText(nf.format(value));
        textField.setCaretPosition(0);
    }
}// CLASSIFICATION: UNCLASSIFIED
