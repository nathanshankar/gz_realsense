#include "gz_dynamic_projector/DynamicProjector.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>

#include <algorithm>
#include <cmath>
#include <random>

using namespace gz;
using namespace dynamic_projector;

// Static member definition
bool DynamicProjector::patternInitialized = false;
std::vector<DotInfo> DynamicProjector::sharedDotPattern;

DynamicProjector::DynamicProjector() = default;

void DynamicProjector::Configure(const sim::Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 sim::EntityComponentManager &_ecm,
                                 sim::EventManager &)
{
    mode = _sdf->Get<std::string>("mode", "mono").first;
    sensor1Topic = _sdf->Get<std::string>("sensor1_topic");
    laserPower = _sdf->Get<int>("laser_power", 100).first;
    hfov = _sdf->Get<double>("hfov", 1.7).first;
    vfov = _sdf->Get<double>("vfov", 1.2).first;
    maxRange = _sdf->Get<double>("max_range", 10.0).first;
    enhanceRgb = _sdf->Get<bool>("enhance_rgb", false).first;
    pattern_type = _sdf->Get<std::string>("pattern_type", "dot").first;

    if (mode == "stereo")
    {
        sensor2Topic = _sdf->Get<std::string>("sensor2_topic");
        disparityOffset = _sdf->Get<float>("disparity_offset", 0.0f).first;
    }

    if (laserPower < 15)
        numDots = 0;
    else
    {
        double dotScale = static_cast<double>(laserPower - 15) / (360.0 - 15.0);
        numDots = _sdf->Get<int>("num_dots", 1000 + static_cast<int>(dotScale * (5000 - 1000))).first;
    }

    node.Subscribe(sensor1Topic + "/image", &DynamicProjector::OnNewImage1, this);
    node.Subscribe(sensor1Topic + "/points", &DynamicProjector::OnNewPointCloud1, this);
    node.Subscribe(sensor1Topic + "/camera_info", &DynamicProjector::OnCameraInfo1, this);

    if (enhanceRgb){
    node.Subscribe("/color/image_raw", &DynamicProjector::OnNewColorImage, this);
    enhancedPub = node.Advertise<msgs::Image>("/color/enhanced");}

    // Use topic naming scheme from user request
    imagePub1 = node.Advertise<msgs::Image>(sensor1Topic + "/ir/image");

    if (mode == "stereo")
    {
        node.Subscribe(sensor2Topic + "/image", &DynamicProjector::OnNewImage2, this);
        node.Subscribe(sensor2Topic + "/points", &DynamicProjector::OnNewPointCloud2, this);
        node.Subscribe(sensor2Topic + "/camera_info", &DynamicProjector::OnCameraInfo2, this);
        imagePub2 = node.Advertise<msgs::Image>(sensor2Topic + "/ir/image");
    }

    GeneratePattern();
    initialized = true;
}

void DynamicProjector::OnNewColorImage(const msgs::Image &msg)
{
    if (msg.pixel_format_type() != msgs::PixelFormatType::RGB_INT8)
        return;

    unsigned int width = msg.width();
    unsigned int height = msg.height();
    size_t totalPixels = static_cast<size_t>(width) * height;

    const unsigned char *src = reinterpret_cast<const unsigned char *>(msg.data().data());

    // First pass: compute min/max luminance
    float minY = 255.0f, maxY = 0.0f;
    std::vector<float> luminances(totalPixels);
    for (size_t i = 0; i < totalPixels; ++i)
    {
        float Y = 0.299f * src[3*i] + 0.587f * src[3*i + 1] + 0.114f * src[3*i + 2];
        luminances[i] = Y;
        minY = std::min(minY, Y);
        maxY = std::max(maxY, Y);
    }

    float range = std::max(1.0f, maxY - minY);
    float scale = 220.0f / range; // cap brightest at 220

    // Prepare output image
    msgs::Image out;
    out.set_width(width);
    out.set_height(height);
    out.set_pixel_format_type(msgs::PixelFormatType::L_INT8);
    out.set_step(width);
    out.mutable_header()->CopyFrom(msg.header());

    std::string *outData = out.mutable_data();
    outData->resize(totalPixels);

    unsigned char *dst = reinterpret_cast<unsigned char *>(outData->data());
    for (size_t i = 0; i < totalPixels; ++i)
    {
        float normY = (luminances[i] - minY) * scale;
        dst[i] = static_cast<unsigned char>(std::clamp(normY, 0.0f, 220.0f));
    }

    // Publish enhanced grayscale
    enhancedPub.Publish(out);
}

void DynamicProjector::GeneratePattern()
{
  if (patternInitialized)
  {
    this->dotPattern = sharedDotPattern;
    return;
  }

  float pattern_width = this->hfov;
  if (this->mode == "stereo")
    pattern_width += std::abs(this->disparityOffset);

  float start_angle = -pattern_width / 2.0f;

  int hCells = std::max(1, static_cast<int>(std::sqrt(this->numDots * (pattern_width / this->vfov))));
  int vCells = std::max(1, static_cast<int>(std::sqrt(this->numDots * (this->vfov / pattern_width))));

  float hStep = pattern_width / hCells;
  float vStep = this->vfov / vCells;
  std::uniform_real_distribution<float> hOffset(-hStep / 2.f, hStep / 2.f);
  std::uniform_real_distribution<float> vOffset(-vStep / 2.f, vStep / 2.f);
  std::uniform_real_distribution<float> shapeFactor(0.7f, 1.0f);

  this->dotPattern.clear();

  int shapeCode = 0;
  if (pattern_type == "dot") shapeCode = 0;
  else if (pattern_type == "oval") shapeCode = 1;
  else if (pattern_type == "rectangle") shapeCode = 2;

  for (int i = 0; i < hCells; ++i)
  {
    for (int j = 0; j < vCells; ++j)
    {
      if (this->dotPattern.size() >= this->numDots) break;
      float h = start_angle + i * hStep + hOffset(this->rng);
      float v = -this->vfov / 2.f + j * vStep + vOffset(this->rng);
      this->dotPattern.push_back(DotInfo{h, v, shapeFactor(rng), shapeCode});
    }
    if (this->dotPattern.size() >= this->numDots) break;
  }

  sharedDotPattern = this->dotPattern;
  patternInitialized = true;
}


void DynamicProjector::OnNewImage1(const msgs::Image &msg)
{
  if (!initialized) return;
  ProcessImage(msg, imagePub1, lastCloud1, true);
}
void DynamicProjector::OnNewPointCloud1(const msgs::PointCloudPacked &msg) { lastCloud1 = msg; }
void DynamicProjector::OnCameraInfo1(const msgs::CameraInfo &msg)
{
    if (!cam1InfoReceived && msg.has_intrinsics())
    {
        for (int i=0; i<9; ++i) this->cam1_K[i] = msg.intrinsics().k(i);
        cam1InfoReceived = true;
    }
}
void DynamicProjector::OnNewImage2(const msgs::Image &msg)
{
  if (!initialized) return;
  ProcessImage(msg, imagePub2, lastCloud2, false);
}
void DynamicProjector::OnNewPointCloud2(const msgs::PointCloudPacked &msg) { lastCloud2 = msg; }
void DynamicProjector::OnCameraInfo2(const msgs::CameraInfo &msg)
{
    if (!cam2InfoReceived && msg.has_intrinsics())
    {
        for (int i=0; i<9; ++i) this->cam2_K[i] = msg.intrinsics().k(i);
        cam2InfoReceived = true;
    }
}

/*** Helper: Compute ambience score from RGB pixels (Y, sigma, warmth) ***/
float DynamicProjector::ComputeAmbienceScoreFromRGB(const unsigned char *src, size_t totalPixels)
{
    if (totalPixels == 0) return 128.0f;

    // Step 1: luminance per pixel Y = 0.299R + 0.587G + 0.114B
    std::vector<float> Ys;
    Ys.reserve(totalPixels);
    double sumY = 0.0;
    for (size_t i = 0; i < totalPixels; ++i)
    {
        unsigned char R = src[3*i];
        unsigned char G = src[3*i + 1];
        unsigned char B = src[3*i + 2];
        float Y = 0.299f * R + 0.587f * G + 0.114f * B;
        Ys.push_back(Y);
        sumY += Y;
    }
    float muY = static_cast<float>(sumY / static_cast<double>(totalPixels));

    // Step 2a: standard deviation of luminance
    double sumSq = 0.0;
    for (size_t i = 0; i < totalPixels; ++i)
    {
        double d = Ys[i] - muY;
        sumSq += d * d;
    }
    float sigmaY = static_cast<float>(std::sqrt(sumSq / static_cast<double>(totalPixels)));
    // clamp sigma to 0-255
    sigmaY = std::min(255.0f, sigmaY);

    // Step 2b: warmth proxy (ratio R>B)
    size_t warmCount = 0;
    for (size_t i = 0; i < totalPixels; ++i)
    {
        unsigned char R = src[3*i];
        unsigned char B = src[3*i + 2];
        if (R > B) ++warmCount;
    }
    float warmthRatio = (totalPixels > 0) ? (static_cast<float>(warmCount) / static_cast<float>(totalPixels)) : 0.5f;
    float warmth = warmthRatio * 255.0f;

    // Step 3: weighted sum -> ambience
    const float alpha = 0.6f; // brightness weight
    const float beta  = 0.3f; // contrast weight
    const float gamma = 0.1f; // warmth weight

    float ambience = alpha * muY + beta * sigmaY + gamma * warmth;
    ambience = std::min(255.0f, std::max(0.0f, ambience));
    return ambience;
}

/*** Helper: Compute ambience score from L (single channel) image ***/
float DynamicProjector::ComputeAmbienceScoreFromL(const unsigned char *src, size_t totalPixels)
{
    if (totalPixels == 0) return 128.0f;
    double sumY = 0.0;
    for (size_t i = 0; i < totalPixels; ++i) sumY += src[i];
    float muY = static_cast<float>(sumY / static_cast<double>(totalPixels));
    double sumSq = 0.0;
    for (size_t i = 0; i < totalPixels; ++i) { double d = src[i] - muY; sumSq += d * d; }
    float sigmaY = static_cast<float>(std::sqrt(sumSq / static_cast<double>(totalPixels)));
    sigmaY = std::min(255.0f, sigmaY);

    // no color info -> assume neutral warmth
    float warmth = 127.5f;

    const float alpha = 0.6f;
    const float beta  = 0.3f;
    const float gamma = 0.1f;

    float ambience = alpha * muY + beta * sigmaY + gamma * warmth;
    ambience = std::min(255.0f, std::max(0.0f, ambience));
    return ambience;
}

void DynamicProjector::ProcessImage(const msgs::Image &msg,
                                    transport::Node::Publisher &pub,
                                    const msgs::PointCloudPacked &cloud,
                                    bool isIr1)
{
  msgs::Image irMsg;
  irMsg.set_width(msg.width());
  irMsg.set_height(msg.height());
  irMsg.set_pixel_format_type(msgs::PixelFormatType::L_INT8);
  irMsg.set_step(msg.width());
  irMsg.mutable_header()->CopyFrom(msg.header());
  
  std::string *imgData = irMsg.mutable_data();
  imgData->resize(static_cast<size_t>(msg.width()) * msg.height());

  unsigned char ambienceScore = 128; // default

  const float gammaPow = 0.6f;
  const float maxBackground = 220.0f;  // keep natural scene capped at light grey

  if (msg.pixel_format_type() == msgs::PixelFormatType::RGB_INT8)
  {
    const unsigned char *src = reinterpret_cast<const unsigned char *>(msg.data().data());
    unsigned char *dst = reinterpret_cast<unsigned char *>(imgData->data());
    size_t totalPixels = static_cast<size_t>(msg.width()) * msg.height();

    // Compute ambience score using provided method (Y, sigma, warmth)
    float ambience = ComputeAmbienceScoreFromRGB(src, totalPixels);
    ambienceScore = static_cast<unsigned char>(std::lround(ambience));

    // Map ambience to boost & dynamic laser:
    // contrastBoost = 1.0 + k * (1 - ambienceNorm)
    const float kBoost = 1.5f; // max extra boost in very dark scenes
    float ambienceNorm = ambience / 255.0f;
    float contrastBoost = 1.0f + kBoost * (1.0f - ambienceNorm);

    for (size_t i = 0; i < totalPixels; ++i)
    {
      float Y = 0.299f * src[3*i] + 0.587f * src[3*i + 1] + 0.114f * src[3*i + 2];
      float boosted = 255.0f * std::pow(std::clamp(Y / 255.0f, 0.0f, 1.0f), gammaPow);
      float finalValue = boosted * contrastBoost;
      dst[i] = static_cast<unsigned char>(std::clamp(finalValue, 0.0f, maxBackground));
    }
  }
  else if (msg.pixel_format_type() == msgs::PixelFormatType::L_INT8)
  {
    const unsigned char *src = reinterpret_cast<const unsigned char *>(msg.data().data());
    unsigned char *dst = reinterpret_cast<unsigned char *>(imgData->data());
    size_t totalPixels = static_cast<size_t>(msg.width()) * msg.height();

    float ambience = ComputeAmbienceScoreFromL(src, totalPixels);
    ambienceScore = static_cast<unsigned char>(std::lround(ambience));

    const float kBoost = 1.5f;
    float ambienceNorm = ambience / 255.0f;
    float contrastBoost = 1.0f + kBoost * (1.0f - ambienceNorm);

    for (size_t i = 0; i < totalPixels; ++i)
    {
      float Y = static_cast<float>(src[i]);
      float boosted = 255.0f * std::pow(std::clamp(Y / 255.0f, 0.0f, 1.0f), gammaPow);
      float finalValue = boosted * contrastBoost;
      dst[i] = static_cast<unsigned char>(std::clamp(finalValue, 0.0f, maxBackground));
    }
  }
  else
  {
    imgData->assign(msg.data().begin(), msg.data().end());
  }

  OverlayDots(irMsg, cloud, isIr1, ambienceScore);

  pub.Publish(irMsg);
}

void DynamicProjector::OverlayDots(gz::msgs::Image &imgMsg,
                                   const gz::msgs::PointCloudPacked &cloud,
                                   bool isIr1,
                                   unsigned char ambienceScore)
{
    unsigned int width = imgMsg.width();
    unsigned int height = imgMsg.height();
    unsigned char *data = reinterpret_cast<unsigned char *>(imgMsg.mutable_data()->data());

    // Laser base scale (0..1) from configured laserPower
    float baseLaserScale = (laserPower < 15) ? 0.0f : (static_cast<float>(laserPower - 15) / (360.0f - 15.0f));

    // Combined scale: if scene is bright, laser should not increase overall brightness.
    // So effectiveLaserScale = baseLaserScale * (1 - ambienceNorm)
    float ambienceNorm = static_cast<float>(ambienceScore) / 255.0f;
    float effectiveLaserScale = baseLaserScale * (1.0f - ambienceNorm);

    // Dot intensity depends on both configured laserPower and whether scene is dark
    // high laserPower & dark scene -> crisp white dots
    // Dots are always pure white in IR
    float dotIntensityF = 255.0f;


    // Dot alpha/opacity: low when laser weak or scene bright
    float dotAlphaBase = 0.2f + effectiveLaserScale * 0.85f; // ~0.2..1.05 -> clamp later
    dotAlphaBase = std::clamp(dotAlphaBase, 0.0f, 1.0f);

    // Dot sharpness influenced by both ambience and laser scale: 
    // - Dark scenes and high laser power -> sharper dots
    // - Bright scenes or low laser power -> blurrier dots
    // Dot sharpness: 
    // - Inner loop: laser power (effectiveLaserScale) controls base sharpness (low power = low sharpness)
    // - Outer loop: ambience (ambienceNorm) modulates the max sharpness (bright scene = less sharp, dark = sharper)
    float baseSharpness = 0.8f + 1.2f * effectiveLaserScale; // 0.8 (min) to 2.0 (max) from laser power
    float maxSharpness = 1.0f + 1.6f * (1.0f - ambienceNorm); // 1.0 (bright) to 2.6 (dark)
    float dotSharpness = std::min(baseSharpness, maxSharpness);

    auto drawDot = [&](int u, int v, float dist, const DotInfo &dot) {
    float baseRadius = std::clamp(6.0f / (dist + 1e-3f), 1.0f, 4.0f);

    // --- CIRCLE ---
    if (dot.shape == 0) {
        float rx = baseRadius;
        float ry = baseRadius;
        for (int dy = -static_cast<int>(ry); dy <= static_cast<int>(ry); ++dy) {
            for (int dx = -static_cast<int>(rx); dx <= static_cast<int>(rx); ++dx) {
                float nxNorm = (float)dx / rx;
                float nyNorm = (float)dy / ry;
                if ((nxNorm * nxNorm + nyNorm * nyNorm) > 1.0f) continue;
                int nx = u + dx; int ny = v + dy;
                if (nx < 0 || nx >= (int)width || ny < 0 || ny >= (int)height) continue;

                float dissolve = (nxNorm * nxNorm + nyNorm * nyNorm);
                float alpha = dotAlphaBase * (1.0f - dissolve);
                alpha = std::clamp(alpha, 0.0f, 1.0f);

                float dotVal = dotIntensityF * dotSharpness;
                float bg = static_cast<float>(data[ny * width + nx]);
                float out = (1.0f - alpha) * bg + alpha * dotVal;
                data[ny * width + nx] = static_cast<unsigned char>(std::clamp(out, 0.0f, 255.0f));
            }
        }
    }

    // --- OVAL ---
    else if (dot.shape == 1) {
        float rx = baseRadius * 1.5f; // wider
        float ry = baseRadius * 0.6f; // flatter
        for (int dy = -static_cast<int>(ry); dy <= static_cast<int>(ry); ++dy) {
            for (int dx = -static_cast<int>(rx); dx <= static_cast<int>(rx); ++dx) {
                float nxNorm = (float)dx / rx;
                float nyNorm = (float)dy / ry;
                if ((nxNorm * nxNorm + nyNorm * nyNorm) > 1.0f) continue;
                int nx = u + dx; int ny = v + dy;
                if (nx < 0 || nx >= (int)width || ny < 0 || ny >= (int)height) continue;

                float dissolve = (nxNorm * nxNorm + nyNorm * nyNorm);
                float alpha = dotAlphaBase * (1.0f - dissolve);
                alpha = std::clamp(alpha, 0.0f, 1.0f);

                float dotVal = dotIntensityF * dotSharpness;
                float bg = static_cast<float>(data[ny * width + nx]);
                float out = (1.0f - alpha) * bg + alpha * dotVal;
                data[ny * width + nx] = static_cast<unsigned char>(std::clamp(out, 0.0f, 255.0f));
            }
        }
    }

    // --- VERTICAL RECTANGLE ---
    else if (dot.shape == 2) {
        int rx = static_cast<int>(baseRadius * 0.6f);  // narrower width
        int ry = static_cast<int>(baseRadius * 1.8f);  // taller height
        for (int dy = -ry; dy <= ry; ++dy) {
            for (int dx = -rx; dx <= rx; ++dx) {
                int nx = u + dx; int ny = v + dy;
                if (nx < 0 || nx >= (int)width || ny < 0 || ny >= (int)height) continue;

                float alpha = dotAlphaBase;
                float dotVal = dotIntensityF * dotSharpness;
                float bg = static_cast<float>(data[ny * width + nx]);
                float out = (1.0f - alpha) * bg + alpha * dotVal;
                data[ny * width + nx] = static_cast<unsigned char>(std::clamp(out, 0.0f, 255.0f));
            }
        }
    }
};


    // --- STEREO LOGIC ---
    if (this->mode == "stereo")
    {
        if (!cam1InfoReceived || !cam2InfoReceived || lastCloud1.width() == 0 || lastCloud2.width() == 0) return;

        float fx1 = static_cast<float>(this->cam1_K[0]), cx1 = static_cast<float>(this->cam1_K[2]),
              fy1 = static_cast<float>(this->cam1_K[4]), cy1 = static_cast<float>(this->cam1_K[5]);
        float fx2 = static_cast<float>(this->cam2_K[0]), cx2 = static_cast<float>(this->cam2_K[2]),
              fy2 = static_cast<float>(this->cam2_K[4]), cy2 = static_cast<float>(this->cam2_K[5]);

        float baseline = this->disparityOffset;
        math::Vector3d cam1_pos(0, baseline / 2.0, 0); // Left eye
        math::Vector3d cam2_pos(0, -baseline / 2.0, 0); // Right eye

        for (const auto &dot : this->dotPattern)
        {
            math::Vector3d p_final_world_frame;
            bool point_found = false;
            float h_in_cam1 = dot.h + baseline / 2.0f;
            float h_in_cam2 = dot.h - baseline / 2.0f;

            auto findIntersection = [&](const msgs::PointCloudPacked& pc, float h, float v, float fx, float cx, float fy, float cy) -> std::tuple<math::Vector3d, bool>
            {
                if (std::abs(h) > hfov / 2.0) return std::make_tuple(math::Vector3d(), false);
                int u = static_cast<int>(fx * std::tan(h) + cx);
                int v_px = static_cast<int>(fy * std::tan(v) + cy);
                if (u < 0 || u >= static_cast<int>(width) || v_px < 0 || v_px >= static_cast<int>(height)) return std::make_tuple(math::Vector3d(), false);

                int cX = std::clamp(static_cast<int>((float)u * pc.width() / width), 0, (int)pc.width() - 1);
                int cY = std::clamp(static_cast<int>((float)v_px * pc.height() / height), 0, (int)pc.height() - 1);
                size_t idx = (cY * pc.width() + cX) * pc.point_step();
                const float* px = reinterpret_cast<const float*>(&pc.data()[idx + pc.field(0).offset()]);
                
                if(std::isfinite(*px)) {
                    math::Vector3d p(*px, *reinterpret_cast<const float*>(&pc.data()[idx + pc.field(1).offset()]), *reinterpret_cast<const float*>(&pc.data()[idx + pc.field(2).offset()]));
                    if(p.X() > 0) return {p, true};
                }
                return std::make_tuple(math::Vector3d(), false);
            };

            auto [p1_cam_frame, found1] = findIntersection(lastCloud1, h_in_cam1, dot.v, fx1, cx1, fy1, cy1);
            auto [p2_cam_frame, found2] = findIntersection(lastCloud2, h_in_cam2, dot.v, fx2, cx2, fy2, cy2);

            if (found1 && found2) {
                math::Vector3d p1_world = p1_cam_frame + cam1_pos;
                math::Vector3d p2_world = p2_cam_frame + cam2_pos;
                p_final_world_frame = (p1_world.Length() < p2_world.Length()) ? p1_world : p2_world;
                point_found = true;
            } else if (found1) {
                p_final_world_frame = p1_cam_frame + cam1_pos;
                point_found = true;
            } else if (found2) {
                p_final_world_frame = p2_cam_frame + cam2_pos;
                point_found = true;
            }
            
            if (!point_found) continue;
            
            if (isIr1) {
                math::Vector3d p_in_cam1_frame = p_final_world_frame - cam1_pos; 
                if (p_in_cam1_frame.X() <= 0) continue;
                int u = static_cast<int>(-fy1 * (p_in_cam1_frame.Y() / p_in_cam1_frame.X()) + cx1);
                int v = static_cast<int>(-fx1 * (p_in_cam1_frame.Z() / p_in_cam1_frame.X()) + cy1);
                if (u >= 0 && u < static_cast<int>(width) && v >= 0 && v < static_cast<int>(height)) {
                    drawDot(u, v, static_cast<float>(p_in_cam1_frame.Length()), dot);
                }
            } else { // isIr2
                math::Vector3d p_in_cam2_frame = p_final_world_frame - cam2_pos;
                if (p_in_cam2_frame.X() <= 0) continue;
                int u = static_cast<int>(-fy2 * (p_in_cam2_frame.Y() / p_in_cam2_frame.X()) + cx2);
                int v = static_cast<int>(-fx2 * (p_in_cam2_frame.Z() / p_in_cam2_frame.X()) + cy2);
                if (u >= 0 && u < static_cast<int>(width) && v >= 0 && v < static_cast<int>(height)) {
                    drawDot(u, v, static_cast<float>(p_in_cam2_frame.Length()), dot);
                }
            }
        }
        return; // End of stereo logic
    }

    // --- MONO LOGIC ---
    if (!cam1InfoReceived || lastCloud1.width() == 0) return;
    
    float fx = static_cast<float>(this->cam1_K[0]), cx = static_cast<float>(this->cam1_K[2]),
          fy = static_cast<float>(this->cam1_K[4]), cy = static_cast<float>(this->cam1_K[5]);

    int cW = lastCloud1.width(), cH = lastCloud1.height(), pStep = lastCloud1.point_step();
    const unsigned char* pcData = reinterpret_cast<const unsigned char*>(lastCloud1.data().data());
    int xOff = lastCloud1.field(0).offset(), yOff = lastCloud1.field(1).offset(), zOff = lastCloud1.field(2).offset();

    for (const auto &dot : this->dotPattern)
    {
        int u = static_cast<int>(fx * std::tan(dot.h) + cx);
        int v = static_cast<int>(fy * std::tan(dot.v) + cy);

        if (u >= 0 && u < static_cast<int>(width) && v >= 0 && v < static_cast<int>(height))
        {
            int cX = std::clamp(static_cast<int>((float)u * cW / width), 0, cW - 1);
            int cY = std::clamp(static_cast<int>((float)v * cH / height), 0, cH - 1);
            size_t idx = (cY * cW + cX) * pStep;
            
            const float* px = reinterpret_cast<const float*>(pcData + idx + xOff);
            const float* py = reinterpret_cast<const float*>(pcData + idx + yOff);
            const float* pz = reinterpret_cast<const float*>(pcData + idx + zOff);

            if (std::isfinite(*px) && std::isfinite(*py) && std::isfinite(*pz)) {
                float dist = std::sqrt((*px)*(*px) + (*py)*(*py) + (*pz)*(*pz));
                if (dist > 0) drawDot(u, v, dist, dot);
            }
        }
    }
}


void DynamicProjector::PostUpdate(const sim::UpdateInfo &, const sim::EntityComponentManager &) {}

GZ_ADD_PLUGIN(DynamicProjector,
              gz::sim::System,
              DynamicProjector::ISystemConfigure,
              DynamicProjector::ISystemPostUpdate)
