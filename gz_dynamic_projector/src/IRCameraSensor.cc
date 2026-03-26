#include "gz_dynamic_projector/IRCameraSensor.hh"

#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <algorithm>

using namespace gz;
using namespace sensors;

IRCameraSensor::IRCameraSensor() = default;
IRCameraSensor::~IRCameraSensor() = default;

bool IRCameraSensor::Load(const sdf::Sensor &_sdf)
{
    if (!CameraSensor::Load(_sdf))
        return false;

    this->width  = this->ImageWidth();
    this->height = this->ImageHeight();

    if (_sdf.Element()->HasElement("hfov"))
        this->hfov = _sdf.Element()->Get<double>("hfov");
    if (_sdf.Element()->HasElement("vfov"))
        this->vfov = _sdf.Element()->Get<double>("vfov");
    if (_sdf.Element()->HasElement("num_dots"))
        this->numDots = _sdf.Element()->Get<unsigned int>("num_dots");

    this->GeneratePattern();

    this->imagePub = this->node.Advertise<gz::msgs::Image>(
            this->Topic() + "/ir");

    gzmsg << "IRCameraSensor loaded: " << this->width << "x" << this->height
                << " with " << this->numDots << " dots" << std::endl;
    return true;
}

bool IRCameraSensor::Update(const std::chrono::steady_clock::duration &_now)
{
    if (!this->HasConnections())
        return false;

    // Setup grayscale image buffer
    this->imageMsg.set_width(this->width);
    this->imageMsg.set_height(this->height);
    this->imageMsg.set_step(this->width);
    this->imageMsg.set_pixel_format_type(msgs::PixelFormatType::L_INT8);

    std::string *data = this->imageMsg.mutable_data();
    data->assign(this->width * this->height, static_cast<char>(0));

    // Overlay pattern
    this->OverlayDots();

    // Stamp
    auto sec  = std::chrono::duration_cast<std::chrono::seconds>(_now).count();
    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(_now).count() % 1000000000;
    auto* stamp = this->imageMsg.mutable_header()->mutable_stamp();
    stamp->set_sec(sec);
    stamp->set_nsec(nsec);

    // Publish
    this->imagePub.Publish(this->imageMsg);

    return true;
}


void IRCameraSensor::GeneratePattern()
{
  std::uniform_real_distribution<float> hDist(-this->hfov/2, this->hfov/2);
  std::uniform_real_distribution<float> vDist(-this->vfov/2, this->vfov/2);

  this->dotPattern.clear();
  for (unsigned int i = 0; i < this->numDots; ++i)
    this->dotPattern.emplace_back(hDist(this->rng), vDist(this->rng));
}

void IRCameraSensor::OverlayDots()
{
  std::string *data = this->imageMsg.mutable_data();
  unsigned char *imgData =
      reinterpret_cast<unsigned char *>(data->data());

  for (const auto &dot : this->dotPattern)
  {
    int u = static_cast<int>((0.5 + dot.first / this->hfov) * this->width);
    int v = static_cast<int>((0.5 + dot.second / this->vfov) * this->height);

    if (u < 0 || u >= static_cast<int>(this->width) ||
        v < 0 || v >= static_cast<int>(this->height))
      continue;

    imgData[v * this->width + u] = 255;  // white dot
  }
}

// Register sensor
GZ_ADD_PLUGIN(IRCameraSensor, gz::sensors::Sensor)
