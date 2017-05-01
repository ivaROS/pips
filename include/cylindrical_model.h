#ifndef CYLINDRICAL_MODEL_H
#define CYLINDRICAL_MODEL_H

#include <hallucinated_robot_model.h>

struct COLUMN_TYPE
{
  cv::Mat image;
  float depth;
};

class CylindricalModel : public HallucinatedRobotModelImpl<cv::Point3d>
{
  private:

    
  public:
    void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
    /*
    virtual bool testCollision(const PoseType pose)
    {
      return testCollision(cv::Point3d(pose.position.x, pose.position.y, pose.position.z));
    }
    */
   
  protected:
  
    bool testCollisionImpl(const cv::Point3d pt);
    
    virtual COLUMN_TYPE getColumn(const cv::Point2d top, const cv::Point2d bottom, double depth);
    std::vector<COLUMN_TYPE> getColumns(const cv::Point3d pt);

/*
    virtual cv::Mat generateHallucinatedRobot(const PoseType pose)
    {
      return generateHallucinatedRobot(cv::Point3d(pose.position.x, pose.position.y, pose.position.z));
    }
    */
    
    virtual cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt);
    
    virtual std::string getName() { return "CylindricalModel"; }

    //cv::Mat getImage(cv_bridge::CvImage::ConstPtr& cv_image_ref);

};


/*
template<typename T>
bool CylindricalModel::checkCollisions(const cv::Mat& image, const std::vector<COLUMN_TYPE>& cols)
{
  if(cols.size() > 0)
  {
    int col_num = cols[0].x;
    for(int i = 0; i < cols.size(); ++i, ++col_num)
    {
      int row_num = cols[i].y;
      int height = cols[i].height;
      float depth = cols[i].depth;
      
      for(int j = 0; j < height; ++j, ++row_num)
      {
        if(
      }

    }
  }
  else
  {
    return false;
  }
}
*/

//transposed version
/*
template<typename T>
bool CylindricalModel::checkCollisions(const cv::Mat& image, const std::vector<COLUMN_TYPE>& rows)
{
  if(rows.size() > 0)
  {
    int row_num = rows[0].x;
    for(int i = 0; i < rows.size(); ++i, ++row_num)
    {
      const T* p = image.ptr<T>(row_num);
      int width = cols[i].height;
      float depth = cols[i].depth;
      
      for(int j = 0; j < height; ++j, ++col_num)
      {
        if(
      }

    }
  }
  else
  {
    return false;
  }
}
*/      
/*

//Could use templates to remove the duplication of this code
    if(image.depth() == CV_32FC1)
    {
      int i,j;
      const float* p;
      for( i = 0; i < nRows; ++i)
      {
          p = image.ptr<float>(i);
          for ( j = 0; j < nCols; ++j)
          {
              if(p[j] < depth)
              {
                return true;
              }
                
          }
      }
    }
    else if (image.depth() == CV_16UC1)
    {
      int i,j;
      const unsigned short int* p;
      for( i = 0; i < nRows; ++i)
      {
          p = image.ptr<unsigned short int>(i);
          for ( j = 0; j < nCols; ++j)
          {
              if(p[j] < depth)
              {
                return true;
              }
                
          }
      }
    }
          
    */

#endif /*  CYLINDRICAL_MODEL_H */
