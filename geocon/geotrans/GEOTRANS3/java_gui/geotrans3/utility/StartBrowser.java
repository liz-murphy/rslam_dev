// CLASSIFICATION: UNCLASSIFIED

/*
 * StartBrowser.java
 *
 * Created on June 14, 2001, 9:02 AM
 *
 * 08/04/2011  K Swanson  BAEts29008  Added mozilla as a browser option
 */

package geotrans3.utility;


import geotrans3.misc.StringHandler;


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
        StringHandler stringHandler = new StringHandler();
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
                try
                {
                    // try firefox
                    browserCommand = "firefox " + url; 
                    Process process = Runtime.getRuntime().exec(browserCommand);  
                }
                catch(java.io.IOException x) 
                {
                    try
                    {
                        // try mozilla
                        browserCommand = "mozilla " + url;
                        Process process = Runtime.getRuntime().exec(browserCommand); 
                    }
                    catch (java.io.IOException ex)
                    {
                        // try netscape
                        browserCommand = "netscape " + url; 
                        Process process = Runtime.getRuntime().exec(browserCommand);  
                    }
                }
            } 
        } 
        catch(java.io.IOException x) 
        { 
            // couldn't start browser 
            stringHandler.displayErrorMsg(parent, "Could not invoke browser, command=" + browserCommand);
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

}

// CLASSIFICATION: UNCLASSIFIED
