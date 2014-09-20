// CLASSIFICATION: UNCLASSIFIED

/*
 * FillList.java
 *
 * Created on April 26, 2001, 1:49 PM
 */

package geotrans3.misc;


import javax.swing.DefaultComboBoxModel;
import geotrans3.jni.*;
import geotrans3.enumerations.ListType;
import geotrans3.exception.CoordinateConversionException;
import geotrans3.misc.StringHandler;


/**
 *
 * @author  amyc
 * @version 
 */
public class FillList extends Object 
{
  private JNIDatumLibrary datumLibrary;
  private JNIEllipsoidLibrary ellipsoidLibrary;
  private StringHandler stringHandler;
  private long datumCount;

  /** Creates new FillList */
  public FillList(JNIDatumLibrary _jniDatumLibrary, JNIEllipsoidLibrary _jniEllipsoidLibrary, javax.swing.JComboBox list, int listType)
  {
    datumLibrary = _jniDatumLibrary;
    ellipsoidLibrary = _jniEllipsoidLibrary;
    stringHandler = new StringHandler();
    datumCount = 0;

    if(listType == ListType.DATUM)
      fillDatumList(list);
    else
      fillEllipsoidList(list);
  }

  private void fillDatumList(javax.swing.JComboBox datumList)
  {
    try
    {
      datumList.setModel(new DefaultComboBoxModel());

      java.lang.String datumCode = "";
      java.lang.String datumName = "";
      datumCount = datumLibrary.getDatumCount();

      for(int i = 0; i < datumCount; i++)
      {
        Info info = datumLibrary.getDatumInfo(i);
        datumList.addItem(info.getCode() + ":  " + info.getName());
      }

      datumList.setSelectedIndex((int) datumLibrary.getDatumIndex("WGE"));
    }
    catch(Exception e)
    {
      stringHandler.displayErrorMsg(new javax.swing.JFrame(), e.getMessage());
    }
  }
    
    
  private void fillEllipsoidList(javax.swing.JComboBox ellipsoidList)
  {
    try
    {
      java.lang.String ellipsoidCode = "";
      java.lang.String ellipsoidName = "";
      long ellipsoidCount = ellipsoidLibrary.getEllipsoidCount();

      for(int i = 0; i < ellipsoidCount; i++)
      {
        Info info = ellipsoidLibrary.getEllipsoidInfo(i);
        ellipsoidList.addItem(info.getCode() + ":  " + info.getName());
      }

      ellipsoidList.setSelectedIndex((int) ellipsoidLibrary.getEllipsoidIndex("WE"));
    }
    catch(CoordinateConversionException e)
    {
      stringHandler.displayErrorMsg(new javax.swing.JFrame(), e.getMessage());
    }
  }
    
  public void addDatum(javax.swing.JComboBox datumList)
  {
    try
    {
      long newDatumCount = datumLibrary.getDatumCount();

      int index = (int) newDatumCount - 1;
      Info info = datumLibrary.getDatumInfo(index);
      datumList.insertItemAt(info.getCode() + ":  " + info.getName(), index);

      datumCount++;
    /*            long newDatumCount = datumLibrary.getDatumCount();
    if (datumCount < newDatumCount)
    {
    java.lang.String datumCode = "";
    java.lang.String datumName = "";
    for (int i = (int)datumCount; i < newDatumCount; i++)
    {
    Info info = datumLibrary.getDatumInfo(i);
    datumList.insertItemAt(info.getCode() + ":  " + info.getName(), i);
    }
    datumCount = newDatumCount;
    }*/
    }
    catch(CoordinateConversionException e)
    {
      stringHandler.displayErrorMsg(new javax.swing.JFrame(), e.getMessage());
    }
  }
    
  public void deleteDatum(javax.swing.JComboBox datumList, int index)
  {
    try
    {
      long newDatumCount = datumLibrary.getDatumCount();

      if(datumCount > newDatumCount)
      {
        datumList.removeItemAt(index);
        datumCount--;
      }
    }
    catch(CoordinateConversionException e)
    {
      stringHandler.displayErrorMsg(new javax.swing.JFrame(), e.getMessage());
    }
  }
}


// CLASSIFICATION: UNCLASSIFIED
