#ifndef CYLINDRICAL_MODEL_T_H
#define CYLINDRICAL_MODEL_T_H

#include <cylindrical_model.h>


class CylindricalModelT : public CylindricalModel
{
  private:

  public:
     CylindricalModelT();
  protected:
  
    //bool testCollisionImpl(const cv::Point3d pt);
    
    //virtual COLUMN_TYPE getColumn(const cv::Point2d top, const cv::Point2d bottom, const float depth);
    //std::vector<COLUMN_TYPE> getColumns(const cv::Point3d pt);
    
    virtual cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt);
    
    //virtual std::string getName() { return "CylindricalModelT"; }
    
    virtual cv::Rect getROIImpl(int x, int y, int width, int height);

    virtual cv::Mat getImage(cv_bridge::CvImage::ConstPtr& cv_image_ref);

};


#endif /*  CYLINDRICAL_MODEL_T_H */
