// CLASSIFICATION: UNCLASSIFIED

package geotrans3.utility;


// This class creates a *.dat file filter
public class DATFileFilter extends javax.swing.filechooser.FileFilter
{
    public boolean accept(java.io.File f)
    {
        return f.getName().toLowerCase().endsWith(".dat") || f.isDirectory();
    }
    public String getDescription()
    {
        return "Data Files (*.dat)";
    }
}

// CLASSIFICATION: UNCLASSIFIED
