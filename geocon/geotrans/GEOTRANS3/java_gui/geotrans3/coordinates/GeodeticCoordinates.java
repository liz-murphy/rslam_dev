// CLASSIFICATION: UNCLASSIFIED

/*
 * GeodeticCoordinates.java
 *
 * Created on April 3, 2007, 1:22 PM
 *
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */

package geotrans3.coordinates;

/**
 *
 * @author comstam
 */
public class GeodeticCoordinates extends CoordinateTuple
{
  private double longitude;
  private double latitude;
  private double height;
  
  /** Creates a new instance of GeodeticCoordinates */
  public GeodeticCoordinates(int coordinateType) 
  {
    super(coordinateType);
    
    longitude = 0;
    latitude = 0;
    height = 0;
  }
  
  
  public GeodeticCoordinates(int coordinateType, double _longitude, double _latitude, double _height) 
  {
    super(coordinateType);
    
    longitude = _longitude;
    latitude = _latitude;
    height = _height;
  }
  
  
  public GeodeticCoordinates(int coordinateType, String _warningMessage, double _longitude, double _latitude, double _height) 
  {
    super(coordinateType, _warningMessage);
    
    longitude = _longitude;
    latitude = _latitude;
    height = _height;
  }
  
  
  public double getLongitude()
  {
    return longitude;
  }
  
  
  public double getLatitude()
  {
    return latitude;
  }
  
  
  public double getHeight()
  {
    return height;
  }
}

// CLASSIFICATION: UNCLASSIFIED
