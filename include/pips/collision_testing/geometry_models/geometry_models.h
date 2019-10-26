#ifndef PIPS_GEOMETRY_MODELS_H
#define PIPS_GEOMETRY_MODELS_H

#include <pips/collision_testing/geometry_models/generic_models.h>
#include <pips/utils/abstract_camera_model.h>
#include <urdf/model.h>
#include <pips/collision_testing/robot_models/column_type.h>

#include <visualization_msgs/Marker.h>

//this is just included to include a bunch of other stuff until I figure out exactly what is needed
#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>

namespace pips
{
namespace collision_testing
{

  namespace image_geometry_models
  {

class GeometryModel
{
  private:
  
  public:
    const std::shared_ptr<const pips::collision_testing::geometry_models::GenericGeometryModel> source_;
    geometry_msgs::Pose pose_;
    int type_id_;
    
    GeometryModel(const std::shared_ptr<const pips::collision_testing::geometry_models::GenericGeometryModel>& source, const geometry_msgs::Pose& pose):
      source_(source),
      pose_(pose),
      type_id_(source->type_id_)
    {}
      
    
    //TODO: possibly use templated free functions rather than inheritance for this functionality 
    virtual std::vector<COLUMN_TYPE> getColumns(const std::shared_ptr<const pips::utils::AbstractCameraModel>& cam_model_, int img_width, int img_height) const = 0;
    
    static COLUMN_TYPE getColumn(const cv::Point2d top, const cv::Point2d bottom, const float depth, int img_width, int img_height)
    {
      //By forming a Rect in this way, doesn't matter which point is the top and which is the bottom.
      cv::Rect_<double> r(top,bottom);
      
      int x = r.tl().x + .00000000001;  // Add a tiny number to handle cases where the process of projecting to ray, finding intersection, and projecting back to the image plane introduces a tiny numerical error, apparently only with smaller values that are odd. The added value will never push the number to the next integer value but is much larger than any error seen so far
      int y = std::floor(r.tl().y);
      int width = 1;
      int height = std::ceil(r.br().y)-y + 1; //ROI's from rectangles are noninclusive on the right/bottom sides, so need to add 1 to include the bottom row
      
      //The only changes needed to use a transposed image are swapping the x and y as well as width and height
      cv::Rect column = cv::Rect(x,y,width,height);
      cv::Rect imageBounds(0,0,img_width, img_height);
      cv::Rect bounded = column & imageBounds;
      
      COLUMN_TYPE col;
      
      col.rect = bounded;
      col.depth = depth;
      
      //std::cout << std::setprecision(16) << "top = " << top << ", bottom = " << bottom << ", Col = " << r << "tl: " << r.tl() << " br: " << r.br() << ", rect=" << column << ", bounded= " << bounded << "\n"; // Debugging printout to troubleshoot incorrect x values for columns. The 'magic number' added above resolved the problem.
      
      return col;
    }
    
};
}

}

}

#endif
