// CLASSIFICATION: UNCLASSIFIED

package mspccs_spreadsheet_tester.gui;

import java.io.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import mspccs_spreadsheet_tester.utility.Directory;

/** basic class to run an application
/*  looks for a wrapper class; if not found, calls the app with default args
*/
public class AppResultsWindow extends JFrame implements ActionListener {

	private JTextArea output = new JTextArea( 30, 10 );

	private JButton btn_close;
	private JButton btn_save;
	private JButton btn_kill;

	private String app_name;

	private Process proc;
	private boolean proc_running = true;
	private boolean text_blocked = false;
        
        private Directory currentDir;
	
	public AppResultsWindow( Process p, String app, String cLine, Directory _currentDir )
	{
                currentDir = _currentDir;
                
		setTitle( app + " " + "results (Status: Running)" );
                
                // Change title bar icon
                java.net.URL url = getClass().getResource("/mspccs_spreadsheet_tester/gui/icons/NGA.gif");
                try
                {
                    java.awt.Image img = createImage((java.awt.image.ImageProducer)url.getContent());
                    setIconImage(img);
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
                
		app_name = new String(app);
		
		Container contentPane = getContentPane();
		contentPane.setLayout( new BoxLayout( contentPane, BoxLayout.Y_AXIS) );
		
		JTextArea commandline = new JTextArea( "Command:  " + app + 
		                                       " " + cLine );
		commandline.setLineWrap(true);

		contentPane.add( commandline );

		JPanel newPanel = new JPanel();	

		btn_kill = new JButton( "Kill Process");
		btn_kill.setActionCommand("Kill");
		btn_kill.addActionListener( this );
	
		btn_close = new JButton("Close Window");
		btn_close.setActionCommand("Close");
		btn_close.addActionListener( this );

		btn_save = new JButton("Save Output");
		btn_save.setActionCommand("Save");
		btn_save.addActionListener( this );
		btn_save.setEnabled( false );

		newPanel.add(btn_kill);
		newPanel.add(btn_close);
		newPanel.add(btn_save);

		contentPane.add( newPanel );
		
		contentPane.add( new JScrollPane( output ));

		proc = p;

		// exit code?		
		ProcessExit procExit = new ProcessExit( proc );

		// any error message? 
		StreamGobbler errorGobbler = new StreamGobbler(proc.getErrorStream(), 
		                                               "ERROR"); 

		// any output? 
		StreamGobbler outputGobbler = new StreamGobbler(proc.getInputStream(), 
		                                                "OUTPUT"); 

		procExit.setPriority(8);
		errorGobbler.setPriority(4);
		outputGobbler.setPriority(4);

		// kick them off 
		errorGobbler.start(); 
		outputGobbler.start();                
		procExit.start(); 
		
		addWindowListener(new WindowAdapter() 
		{
		       public void windowClosing(WindowEvent e) 
			   {
			   		proc.destroy();
					proc_running = false;
			   		dispose();
		       }
		});

		pack();
		setSize(800, 600);
		Dimension dim = getToolkit().getScreenSize();
		setLocation(dim.width/2 - getWidth()/2,
		            dim.height/2 - getHeight()/2);
		setVisible(true);			
	}
	
	public void actionPerformed(java.awt.event.ActionEvent e)
	{		 
		if(e.getActionCommand().equals("Kill"))
		{
			proc.destroy();
			proc_running = false;
			btn_kill.setEnabled( false );
			btn_save.setEnabled( true );
			setTitle( app_name + " " + "results (Status: Killed)" );
		}
		else if(e.getActionCommand().equals("Close"))
		{
			proc.destroy();
			proc_running = false;
			dispose();
		}
		else if(e.getActionCommand().equals("Save"))
		{

			JFileChooser fc = new JFileChooser(currentDir.getDirectory());
			//fc.setCurrentDirectory( new File(".") );
			fc.setDialogTitle( "Select file to save to..." );

			int result = fc.showSaveDialog( this );

			if(result == fc.APPROVE_OPTION)
			{
		
				File file = fc.getSelectedFile();

				try
				{				
					if( file.exists() )
					{
					 	file.delete();
					}
					file.createNewFile();
					FileWriter out = new FileWriter( file );
					output.write( out );
          out.close();
					JOptionPane.showMessageDialog( null,
					                               "Output saved.");
				}
				catch( Exception doh )
				{
					System.out.println( doh );
				}
			}
		}
	}
	

	class StreamGobbler extends Thread 
	{ 
	    InputStream is; 
	    String type; 

	    StreamGobbler(InputStream is, String type) 
	    { 
	        this.is = is; 
	        this.type = type; 
	    } 

	    public void run() 
	    { 
	        try 
	        { 
	            InputStreamReader isr = new InputStreamReader(is); 
				// this is really big because i believe we are overrunning 
				// it on SGI
	            BufferedReader br = new BufferedReader(isr, 900000);
				String checker = "Reading";
				String line = null;
				
				while ((line = br.readLine()) != null)
				{
					//System.out.println(line);
					if (!(line.regionMatches(0, checker, 0, 7)))
					{
						output.append((line + "\n"));
						output.setCaretPosition(output.getText().length());
					}
					//if(line.lastIndexOf('\n') != -1)
					//{
					//	output.append(line);
					//	System.out.println(line);
					//	output.setCaretPosition(output.getText().length());
					//	
					//}
				}
				
				 
				//int returnval;
				//char input[] = new char[1];

				// let this run until the process ends
				//while(proc_running)
				//{
				//	returnval = br.read(input, 0, 1);
				//	if(returnval == 1)
				//	{
				//		output.append( new String(input) );
//
				//		if(input[0] == '\n')
				//		{
				//			output.setCaretPosition(output.getText().length());
				//		}
				//	}
				//}

				// System.out.println(type + " exited proc running stage");

				// flush the last of the buffers
				//while((returnval = br.read(input, 0, 1)) == 1)
				//{
				//	output.append( new String(input) );
//
				//	if(input[0] == '\n')
				//	{
				//		output.setCaretPosition(output.getText().length());
				//	}
				//}
	        } 
			catch (IOException ioe) 
	        { 
	            ioe.printStackTrace(); 
	        } 
	    } 
	} 
	
	class ProcessExit extends Thread 
	{ 
		Process proc;

	    ProcessExit(Process p) 
	    { 
	        this.proc = p; 
	    } 

	    public void run() 
	    {
			int exitCode = -1;
			try
			{
				exitCode = proc.waitFor();
				// System.out.println("Process ended");
				proc_running = false;
				btn_kill.setEnabled( false );
				btn_save.setEnabled( true );
				setTitle( app_name + " " + "results (Status: Completed)" );
			}
			catch ( Exception e )
			{
				e.printStackTrace();
				proc_running = false;
			}
	    }
	}
}

// CLASSIFICATION: UNCLASSIFIED
