#ifndef GZ_DYNAMIC_PROJECTOR_DYNAMICPROJECTOR_HH_
#define GZ_DYNAMIC_PROJECTOR_DYNAMICPROJECTOR_HH_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <vector>
#include <string>
#include <random>
#include <array>
#include <chrono>
namespace gz
{
namespace dynamic_projector
{

/// \brief Holds the angular information for a single projected dot.
struct DotInfo
{
  float h;             ///< Horizontal angle in radians
  float v;             ///< Vertical angle in radians
  float ellipseFactor; ///< Elliptical shape factor (dot/oval)
  int shape;           ///< 0 = dot, 1 = oval, 2 = square
};

class DynamicProjector
    : public sim::System,
      public sim::ISystemConfigure,
      public sim::ISystemPostUpdate
{
public:
  DynamicProjector();

  // System overrides
  void Configure(const sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 sim::EntityComponentManager &_ecm,
                 sim::EventManager &_eventMgr) override;

  void PostUpdate(const sim::UpdateInfo &_info,
                  const sim::EntityComponentManager &_ecm) override;

private:
  // Callbacks
  void OnNewImage1(const msgs::Image &msg);
  void OnNewPointCloud1(const msgs::PointCloudPacked &msg);
  void OnCameraInfo1(const msgs::CameraInfo &msg);

  void OnNewImage2(const msgs::Image &msg);
  void OnNewPointCloud2(const msgs::PointCloudPacked &msg);
  void OnCameraInfo2(const msgs::CameraInfo &msg);
  void OnNewColorImage(const msgs::Image &msg);
  
  // Core processing
  void ProcessImage(const msgs::Image &msg,
                    transport::Node::Publisher &pub,
                    const msgs::PointCloudPacked &cloud,
                    bool isIr1);

  // Note: OverlayDots now accepts an ambience score (0-255)
  void OverlayDots(gz::msgs::Image &imgMsg,
                   const msgs::PointCloudPacked &cloud,
                   bool isIr1,
                   unsigned char ambienceScore);
  
  void GeneratePattern();

  // Helper to compute ambience from RGB/L image
  float ComputeAmbienceScoreFromRGB(const unsigned char *src, size_t totalPixels);
  float ComputeAmbienceScoreFromL(const unsigned char *src, size_t totalPixels);

  // SDF parameters
  std::string mode{"mono"};
  std::string pattern_type{"dot"};
  bool enhanceRgb{false};
  std::string sensor1Topic;
  std::string sensor2Topic;
  float disparityOffset{0.0f};
  int numDots{5000};
  int laserPower{100};
  double hfov{1.7};
  double vfov{1.2};
  double maxRange{10.0};

  // Transport
  transport::Node node;
  transport::Node::Publisher imagePub1;
  transport::Node::Publisher imagePub2;
  transport::Node::Publisher enhancedPub;
  
  // Cached point clouds and camera info
  msgs::PointCloudPacked lastCloud1;
  msgs::PointCloudPacked lastCloud2;
  bool cam1InfoReceived{false};
  bool cam2InfoReceived{false};
  std::array<double, 9> cam1_K{};
  std::array<double, 9> cam2_K{};

  // Random number generator for pattern
  std::mt19937 rng{static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count())};
  bool initialized{false};
  
  // Static flag to ensure pattern is generated only once
  static bool patternInitialized;
  
  // A single, unified pattern generated from the virtual center emitter
  static std::vector<DotInfo> sharedDotPattern;
  std::vector<DotInfo> dotPattern;
};

}  // namespace dynamic_projector
}  // namespace gz

#endif // GZ_DYNAMIC_PROJECTOR_DYNAMICPROJECTOR_HH_
