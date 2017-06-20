
#include <pips/collision_testing/robot_models/cylindrical_model_t.h>

  /* Transposes are only needed when:
      1. Setting the reference depth image
      2. Generating ROI Mats for columns
      3. Returning the geberated image for visualization
      
  */
  
  CylindricalModelT::CylindricalModelT()
  {
   name_ = "CylindricalModelT";
  }
  
  cv::Mat CylindricalModelT::generateHallucinatedRobotImpl(const cv::Point3d pt)
  {
    cv::Mat viz = CylindricalModel::generateHallucinatedRobotImpl(pt);

    cv::Mat viz_t = viz.t();
  
    return viz_t;
  }

  
  //Transposed version
  cv::Rect CylindricalModelT::getColumnRect(int x, int y, int width, int height)
  {
    cv::Rect column(y,x,height,width);
    return column;
  }
  


  cv::Mat CylindricalModelT::getImageImpl(const cv::Mat& image)
  { 
    cv::Mat transposed = image.t();
    return transposed;
  }



