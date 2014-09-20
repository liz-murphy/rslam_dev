
// CLASSIFICATION: UNCLASSIFIED
/*
 * SaveSettings.java
 *
 * Created on September 20, 2007, 9:05 AM
 *
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */

package geotrans3.misc;


import geotrans3.gui.*;
import geotrans3.misc.FormatOptions;
import geotrans3.enumerations.CoordinateType;


/**
 *
 * @author comstam
 */
public class SaveSettings
{
  private MSP_GEOTRANS3 parent;
  private java.io.File file;
  private MasterPanel upperMasterPanel;
  private MasterPanel lowerMasterPanel;
  private java.io.FileWriter defaultFile;
  private int projectionType = 0;
  private FormatOptions joptions;
  
  
  /** Creates a new instance of SaveSettings */
  public SaveSettings(MSP_GEOTRANS3 _parent, java.io.File _file, MasterPanel _upperMasterPanel, MasterPanel _lowerMasterPanel, FormatOptions _joptions)
  {
    parent = _parent;
    file = _file;
    upperMasterPanel = _upperMasterPanel;
    lowerMasterPanel = _lowerMasterPanel;
    joptions = _joptions;
    
    writeDefaults();
  }
  
  
  private void writeDefaults()
  {
    try
    {
      defaultFile = new java.io.FileWriter(file);

      String contents = "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\r\n";
      contents += "<defaults>\r\n";
      defaultFile.write(contents, 0, contents.length());
      writeOptions();
      contents = "  <source>\r\n";
      contents += "    <datum>\r\n";
      contents += "      <index>" + upperMasterPanel.getDatumIndex()+ "</index>\r\n";
      contents += "      <code>" + upperMasterPanel.getDatumCode() + "</code>\r\n";
      contents += "    </datum>\r\n";
      contents += "    <projection>\r\n";
      projectionType = upperMasterPanel.getProjectionType();
      contents += "      <index>" + projectionType + "</index>\r\n";
      contents += "      <code>" + upperMasterPanel.getProjectionCode() + "</code>\r\n";
      contents += "    </projection>\r\n";
      defaultFile.write(contents, 0, contents.length());
      writeParameters(upperMasterPanel); 
      contents = "  </source>\r\n";
      contents += "  <target>\r\n";
      contents += "    <datum>\r\n";
      contents += "      <index>" + lowerMasterPanel.getDatumIndex()+ "</index>\r\n";
      contents += "      <code>" + (lowerMasterPanel.getDatumCode()) + "</code>\r\n";
      contents += "    </datum>\r\n";
      contents += "    <projection>\r\n";
      projectionType = lowerMasterPanel.getProjectionType();
      contents += "      <index>" + projectionType + "</index>\r\n";
      contents += "      <code>" + lowerMasterPanel.getProjectionCode() + "</code>\r\n";
      contents += "    </projection>\r\n";
      defaultFile.write(contents, 0, contents.length());
      writeParameters(lowerMasterPanel); 
      contents = "  </target>\r\n";
      contents += "</defaults>\r\n";
      defaultFile.write(contents, 0, contents.length());
      defaultFile.flush();
      defaultFile.close();
      
      javax.swing.JOptionPane.showMessageDialog(parent, "Settings saved!", "Save", javax.swing.JOptionPane.INFORMATION_MESSAGE);
    }
    catch(java.io.IOException ioex)
    {
      javax.swing.JOptionPane.showMessageDialog(parent, "Error saving settings!", "Error", javax.swing.JOptionPane.ERROR_MESSAGE);
    }    
  }  
  
  
  private void writeParameters(MasterPanel masterPanel)
  {
    try
    {
      String contents = "    <parameters>\r\n";
      
      switch(projectionType)
      {
        case CoordinateType.ECKERT4:
        case CoordinateType.ECKERT6:
        case CoordinateType.MILLER:
        case CoordinateType.MOLLWEIDE:
        case CoordinateType.SINUSOIDAL:
        case CoordinateType.GRINTEN:
          contents += "      <centralMeridian>" + masterPanel.get3ParamCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.AZIMUTHAL:
        case CoordinateType.BONNE:
        case CoordinateType.CASSINI:
        case CoordinateType.CYLEQA:
        case CoordinateType.GNOMONIC:
        case CoordinateType.ORTHOGRAPHIC:
        case CoordinateType.POLYCONIC:
        case CoordinateType.STEREOGRAPHIC:
          contents += "      <centralMeridian>" + masterPanel.get4ParamCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <originLatitude>" + masterPanel.get4ParamOriginLatitude() + "</originLatitude>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.EQDCYL:
          contents += "      <centralMeridian>" + masterPanel.get4ParamCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <standardParallel>" + masterPanel.get4ParamOriginLatitude() + "</standardParallel>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.LAMBERT_1:
          contents += "      <centralMeridian>" + masterPanel.get5ParamCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <originLatitude>" + masterPanel.get5ParamOriginLatitude() + "</originLatitude>\r\n";
          contents += "      <scaleFactor>" + masterPanel.get5ParamScaleFactor() + "</scaleFactor>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.TRCYLEQA:
        case CoordinateType.TRANMERC:
          contents += "      <centralMeridian>" + masterPanel.get5ParamCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <originLatitude>" + masterPanel.get5ParamOriginLatitude() + "</originLatitude>\r\n";
          contents += "      <scaleFactor>" + masterPanel.get5ParamScaleFactor() + "</scaleFactor>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.MERCATOR_SP:
          contents += "      <centralMeridian>" + masterPanel.get5ParamCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <standardParallel>" + masterPanel.get5ParamOriginLatitude() + "</standardParallel>\r\n";
          contents += "      <scaleFactor>" + masterPanel.get5ParamScaleFactor() + "</scaleFactor>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.MERCATOR_SF:
          contents += "      <centralMeridian>" + masterPanel.get4ParamCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <scaleFactor>" + masterPanel.get4ParamOriginLatitude() + "</scaleFactor>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.ALBERS:
        case CoordinateType.LAMBERT_2:
          contents += "      <centralMeridian>" + masterPanel.get6ParamCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <originLatitude>" + masterPanel.get6ParamOriginLatitude() + "</originLatitude>\r\n";
          contents += "      <standardParallel1>" + masterPanel.get6ParamStandardParallel1() + "</standardParallel1>\r\n";
          contents += "      <standardParallel2>" + masterPanel.get6ParamStandardParallel2() + "</standardParallel2>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.GEODETIC:
          contents += "      <heightType>" + masterPanel.getGeodeticHeight() + "</heightType>\r\n";
          break;
        case CoordinateType.LOCCART:
          contents += "      <originLongitude>" + masterPanel.getOriginLongitude() + "</originLongitude>\r\n";
          contents += "      <originLatitude>" + masterPanel.getOriginLatitude() + "</originLatitude>\r\n";
          contents += "      <originHeight>" + masterPanel.getOriginHeight() + "</originHeight>\r\n";
          contents += "      <orientation>" + masterPanel.getOrientation() + "</orientation>\r\n";
          break;
        case CoordinateType.NEYS:
          contents += "      <centralMeridian>" + masterPanel.getNeysCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <originLatitude>" + masterPanel.getNeysOriginLatitude() + "</originLatitude>\r\n";
          contents += "      <standardParallel1>" + masterPanel.getNeysStandardParallel1() + "</standardParallel1>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.OMERC:
          contents += "      <originLatitude>" + masterPanel.getOmercOriginLatitude() + "</originLatitude>\r\n";
          contents += "      <scaleFactor>" + masterPanel.getOmercScaleFactor() + "</scaleFactor>\r\n";
          contents += "      <longitude1>" + masterPanel.getOmercLongitude1() + "</longitude1>\r\n";
          contents += "      <latitude1>" + masterPanel.getOmercLatitude1() + "</latitude1>\r\n";
          contents += "      <longitude2>" + masterPanel.getOmercLongitude2() + "</longitude2>\r\n";
          contents += "      <latitude2>" + masterPanel.getOmercLatitude2() + "</latitude2>\r\n";
          contents += "      <falseEasting>" + masterPanel.getOmercFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getOmercFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.POLARSTEREO_SP:
          contents += "      <centralMeridian>" + masterPanel.get4ParamCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <standardParallel>" + masterPanel.get4ParamOriginLatitude() + "</standardParallel>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.POLARSTEREO_SF:
          contents += "      <centralMeridian>" + masterPanel.getPS_SFCentralMeridian() + "</centralMeridian>\r\n";
          contents += "      <scaleFactor>" + masterPanel.getPS_SFScaleFactor() + "</scaleFactor>\r\n";
          contents += "      <hemisphere>" + masterPanel.getRow1Hemisphere() + "</hemisphere>\r\n";
          contents += "      <falseEasting>" + masterPanel.getFalseEasting() + "</falseEasting>\r\n";
          contents += "      <falseNorthing>" + masterPanel.getFalseNorthing() + "</falseNorthing>\r\n";
          break;
        case CoordinateType.UTM:
          contents += "      <override>" + masterPanel.getOverride() + "</override>\r\n";
          contents += "      <zone>" + masterPanel.getZone() + "</zone>\r\n";
          break;
         
      }
      contents += "    </parameters>\r\n";
      defaultFile.write(contents, 0, contents.length());
    }
    catch(java.io.IOException ioex)
    {
      javax.swing.JOptionPane.showMessageDialog(parent, "Error saving settings!", "Error", javax.swing.JOptionPane.ERROR_MESSAGE);
    }    
  }
  
  
  void writeOptions()
  {
    try
    {
      String contents = "  <options>\r\n";
      
      contents += "    <units>\r\n";
      contents += "      <index>" + joptions.getUnits() + "</index>\r\n";
      contents += "      <value>" + joptions.getUnitsString() + "</value>\r\n";
      contents += "    </units>\r\n";
      
      contents += "    <separator>\r\n";
      contents += "      <index>" + joptions.getSeparatorIndex() + "</index>\r\n";
      contents += "      <value>" + joptions.getSeparator() + "</value>\r\n";
      contents += "    </separator>\r\n";
      
      contents += "    <signHemisphere>\r\n";
      contents += "      <index>" + joptions.getSignHemi() + "</index>\r\n";
      contents += "      <value>" + joptions.getSignHemiString() + "</value>\r\n";
      contents += "    </signHemisphere>\r\n";
      
      contents += "    <longitudeRange>\r\n";
      contents += "      <index>" + joptions.getRange() + "</index>\r\n";
      contents += "      <value>" + joptions.getRangeString() + "</value>\r\n";
      contents += "    </longitudeRange>\r\n";
      
      contents += "    <precision>\r\n";
      contents += "      <index>" + joptions.getPrecision() + "</index>\r\n";
      contents += "      <value>" + joptions.getPrecisionString() + "</value>\r\n";
      contents += "    </precision>\r\n";
      
      contents += "    <leadingZeros>\r\n";
      contents += "      <value>" + joptions.getLeadingZeros() + "</value>\r\n";
      contents += "    </leadingZeros>\r\n";
      
      contents += "  </options>\r\n";   
      defaultFile.write(contents, 0, contents.length());
    }
    catch(java.io.IOException ioex)
    {
      javax.swing.JOptionPane.showMessageDialog(parent, "Error saving settings!", "Error", javax.swing.JOptionPane.ERROR_MESSAGE);
    }    
  }
}

// CLASSIFICATION: UNCLASSIFIED
