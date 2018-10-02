#include <image_geometry/pinhole_camera_model.h>

namespace pips
{
  namespace utils
  {

    class PinholeCameraModel : public image_geometry::PinholeCameraModel
    {
    public:
      cv::Point2d project3dToPixel(const cv::Point3d& xyz) const
      {
        assert( initialized() );
        assert(P_(2, 3) == 0.0); // Calibrated stereo cameras should be in the same plane

        // [U V W]^T = P * [X Y Z 1]^T
        // u = U/W
        // v = V/W
        cv::Point2d uv_rect;
        uv_rect.x = (fx()*xyz.x + Tx()) / xyz.z + cx();
        uv_rect.y = (fy()*xyz.y + Ty()) / xyz.z + cy();
        return uv_rect;
      }

      cv::Point3d projectPixelTo3dRay(const cv::Point2d& uv_rect) const
      {
        assert( initialized() );

        cv::Point3d ray;
        ray.x = (uv_rect.x - cx() - Tx()) / fx();
        ray.y = (uv_rect.y - cy() - Ty()) / fy();
        ray.z = 1.0;
        return ray;
      }

    };

  }
}

namespace pips
{
  namespace utils
  {
    template<typename T>
    struct Point2
    {
    public:
      T x, y;
      constexpr Point2() {}
      
      constexpr Point2(T x, T y) : 
        x(x),
        y(y)
        {}
    };
    typedef Point2<double> Point2d;
    
    template<typename T>
    struct Point3
    {
    public:
      T x, y, z;
      constexpr Point3(T x, T y, T z) : 
      x(x),
      y(y),
      z(z)
      {}
    };
    typedef Point3<double> Point3d;
    
    
    
    class ConstPinholeCameraModel
    {
      double p00,p11,p02,p12,p03,p13;
      
    private:
      
      constexpr inline double fx() const { return p00; }
      constexpr inline double fy() const { return p11; }
      constexpr inline double cx() const { return p02; }
      constexpr inline double cy() const { return p12; }
      constexpr inline double Tx() const { return p03; }
      constexpr inline double Ty() const { return p13; }
      
    public:
      
      constexpr ConstPinholeCameraModel(double p00, double p01, double p02, double p03, double p10, double p11, double p12,  double p13, double p20, double p21, double p22, double p23) :
        p00(p00),
        p11(p11),
        p02(p02),
        p12(p12),
        p03(p03),
        p13(p13)
      {
        
      }
      
      cv::Point2d project3dToPixel(const cv::Point3d& xyz) const
      {
        // [U V W]^T = P * [X Y Z 1]^T
        // u = U/W
        // v = V/W
        return cv::Point2d((fx()*xyz.x + Tx()) / xyz.z + cx(),(fy()*xyz.y + Ty()) / xyz.z + cy());
      }
      
      cv::Point3d projectPixelTo3dRay(const cv::Point2d& uv_rect) const
      {        
        return cv::Point3d((uv_rect.x - cx() - Tx()) / fx(), (uv_rect.y - cy() - Ty()) / fy(), 1.0);
      }
      
      constexpr Point2d project3dToPixel(const Point3d& xyz) const
      {
        // [U V W]^T = P * [X Y Z 1]^T
        // u = U/W
        // v = V/W
        return Point2d((fx()*xyz.x + Tx()) / xyz.z + cx(),(fy()*xyz.y + Ty()) / xyz.z + cy());
      }
      
      constexpr Point3d projectPixelTo3dRay(const Point2d& uv_rect) const
      {        
        return Point3d((uv_rect.x - cx() - Tx()) / fx(), (uv_rect.y - cy() - Ty()) / fy(), 1.0);
      }
      
//       constexpr cv::Point2d PinholeCameraModel::project3dToPixel(const cv::Point3d& xyz) const
//       {
//         assert(P_(2, 3) == 0.0); // Calibrated stereo cameras should be in the same plane
//         
//         // [U V W]^T = P * [X Y Z 1]^T
//         // u = U/W
//         // v = V/W
//         cv::Point2d uv_rect;
//         uv_rect.x = (fx()*xyz.x + Tx()) / xyz.z + cx();
//         uv_rect.y = (fy()*xyz.y + Ty()) / xyz.z + cy();
//         return uv_rect;
//       }
//       
//       constexpr cv::Point3d PinholeCameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect) const
//       {        
//         cv::Point3d ray;
//         ray.x = (uv_rect.x - cx() - Tx()) / fx();
//         ray.y = (uv_rect.y - cy() - Ty()) / fy();
//         ray.z = 1.0;
//         return ray;
//       }
      
    };
    
    class NonConstPinholeCameraModel
    {
      double p00,p11,p02,p12,p03,p13;
      
    private:
      
      inline double fx() const { return p00; }
      inline double fy() const { return p11; }
      inline double cx() const { return p02; }
      inline double cy() const { return p12; }
      inline double Tx() const { return p03; }
      inline double Ty() const { return p13; }
      
    public:
      
      NonConstPinholeCameraModel(double p00, double p01, double p02, double p03, double p10, double p11, double p12,  double p13, double p20, double p21, double p22, double p23) :
      p00(p00),
      p11(p11),
      p02(p02),
      p12(p12),
      p03(p03),
      p13(p13)
      {
        
      }
      
      cv::Point2d project3dToPixel(const cv::Point3d& xyz) const
      {
        // [U V W]^T = P * [X Y Z 1]^T
        // u = U/W
        // v = V/W
        return cv::Point2d((fx()*xyz.x + Tx()) / xyz.z + cx(),(fy()*xyz.y + Ty()) / xyz.z + cy());
      }
      
      cv::Point3d projectPixelTo3dRay(const cv::Point2d& uv_rect) const
      {        
        return cv::Point3d((uv_rect.x - cx() - Tx()) / fx(), (uv_rect.y - cy() - Ty()) / fy(), 1.0);
      }
      
      Point2d project3dToPixel(const Point3d& xyz) const
      {
        // [U V W]^T = P * [X Y Z 1]^T
        // u = U/W
        // v = V/W
        return Point2d((fx()*xyz.x + Tx()) / xyz.z + cx(),(fy()*xyz.y + Ty()) / xyz.z + cy());
      }
      
      Point3d projectPixelTo3dRay(const Point2d& uv_rect) const
      {        
        return Point3d((uv_rect.x - cx() - Tx()) / fx(), (uv_rect.y - cy() - Ty()) / fy(), 1.0);
      }
      
    };
    
  }
}
