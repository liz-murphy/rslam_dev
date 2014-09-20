// CLASSIFICATION: UNCLASSIFIED
package mspccs_spreadsheet_tester.utility;


/*
 * StartBrowser.java
 *
 * Created on June 14, 2001, 9:02 AM
 */


/**
 *
 * @author  amyc
 * @version 
 */
public class StartBrowser extends Object {

    /** Creates new StartBrowser */
    public StartBrowser() {
    }
 
     // The url must start with either "http://" or 
     //"file://". 
      
    public static void displayURL(java.awt.Component parent, String url) 
    { 
        boolean windows = isWindowsPlatform(); 
        String browserCommand = null; 
        try 
        { 
            if (windows) 
            { 
                // browserCommand = 'rundll32 url.dll,FileProtocolHandler http://...' 
                browserCommand = WIN_PATH + " " + WIN_FLAG + " " + url; 
                Process process = Runtime.getRuntime().exec(browserCommand); 
            } 
            else 
            { 
                // browserCommand = 'netscape http://www.javaworld.com' 
                browserCommand = UNIX_PATH + " " + url; 
                Process process = Runtime.getRuntime().exec(browserCommand); 
            } 
        } 
        catch(java.io.IOException x) 
        { 
            // couldn't start browser 
            javax.swing.JOptionPane.showMessageDialog(parent, "Could not invoke browser, command=" + browserCommand, "Error", javax.swing.JOptionPane.ERROR_MESSAGE);
        } 
    } 

    //Determine if operating system is windows
    public static boolean isWindowsPlatform() 
    { 
        String os = System.getProperty("os.name"); 

        if ( os != null && os.startsWith(WIN_ID)) 
            return true; 
        else 
            return false; 
    } 

    // Used to identify the windows platform. 
    private static final String WIN_ID = "Windows"; 

    // Default browser under windows. 
    private static final String WIN_PATH = "rundll32"; 

    // Flag to display a url. 
    private static final String WIN_FLAG = "url.dll,FileProtocolHandler"; 

    // Default browser under unix. 
    private static final String UNIX_PATH = "netscape"; 
   
}
// CLASSIFICATION: UNCLASSIFIED
