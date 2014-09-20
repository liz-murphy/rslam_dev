// CLASSIFICATION: UNCLASSIFIED

/*
 * Info.java
 *
 * Created on April 3, 2007, 10:26 AM
 *
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */

package geotrans3.misc;

/**
 *
 * @author comstam
 */
public class Info 
{
  private java.lang.String code;
  private java.lang.String name;
  private java.lang.String datumEllipsoidCode;
  
  /** Creates a new instance of Info */
  public Info() 
  {
    code = "";
    name = "";
    datumEllipsoidCode = "";
  }
  
  
  public Info(java.lang.String _code, java.lang.String _name) 
  {
    code = _code;
    name = _name;
    datumEllipsoidCode = "";
  }
  
  
  public Info(java.lang.String _code, java.lang.String _name, java.lang.String _datumEllipsoidCode) 
  {
    code = _code;
    name = _name;
    datumEllipsoidCode = _datumEllipsoidCode;
  }
  
  
  public java.lang.String getCode()
  {
    return code;
  }
  
  
  public java.lang.String getName()
  {
    return name;
  }
  
  
  public java.lang.String getDatumEllipsoidCode()
  {
    return datumEllipsoidCode;
  }
}

// CLASSIFICATION: UNCLASSIFIED
