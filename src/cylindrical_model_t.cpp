
#include <pips/collision_testing/robot_models/cylindrical_model_t.h>

  /* Transposes are only needed when:
      1. Setting the reference depth image
      2. Generating ROI Mats for columns
      3. Returning the geberated image for visualization
      
  */
  

  

  
  //Transposed version
  cv::Rect CylindricalModelT::getROIImpl(int x, int y, int width, int height)
  {
    cv::Rect column(y,x,height,width);
    return column;
  }
  





