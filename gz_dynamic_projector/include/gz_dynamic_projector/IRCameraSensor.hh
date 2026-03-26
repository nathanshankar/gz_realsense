#ifndef GZ_SENSORS_IRCAMERASENSOR_HH_
#define GZ_SENSORS_IRCAMERASENSOR_HH_

#include <gz/sensors/CameraSensor.hh>
#include <gz/msgs/image.pb.h>
#include <random>
#include <gz/transport/Node.hh>  // <-- needed
#include <gz/msgs/time.pb.h> 

namespace gz
{
namespace sensors
{

/// \brief IR camera with synthetic dot emitter pattern
class IRCameraSensor : public CameraSensor
{
  public: IRCameraSensor();

  public: virtual ~IRCameraSensor();

  /// \brief Load parameters
  public: virtual bool Load(const sdf::Sensor &_sdf) override;

  /// \brief Update and publish grayscale dot-pattern image
  public: virtual bool Update(const std::chrono::steady_clock::duration &_now) override;

  /// \brief Return last message
  public: const gz::msgs::Image &ImageMsg() const { return this->imageMsg; }

  private: void GeneratePattern();

  private: void OverlayDots();

  private: gz::msgs::Image imageMsg;

  private: unsigned int width{640};

  private: unsigned int height{480};

  private: double hfov{1.5};

  private: double vfov{1.2};

  private: unsigned int numDots{5000};

  private: gz::transport::Node::Publisher imagePub;
  private: gz::transport::Node node;
  private: std::vector<std::pair<float,float>> dotPattern;

  private: std::mt19937 rng{std::random_device{}()};
};

}
}
#endif
