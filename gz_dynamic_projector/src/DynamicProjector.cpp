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
    depthTopic = _sdf->Get<std::string>("depth_topic", "").first;
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

    // Subscribe to images - check both possible naming conventions (topic and topic/image)
    node.Subscribe(sensor1Topic, &DynamicProjector::OnNewImage1, this);
    node.Subscribe(sensor1Topic + "/image", &DynamicProjector::OnNewImage1, this);
    // node.Subscribe(sensor1Topic + "/points", &DynamicProjector::OnNewPointCloud1, this);
    node.Subscribe(sensor1Topic + "/camera_info", &DynamicProjector::OnCameraInfo1, this);

    if (!depthTopic.empty())
    {
        std::string fullDepthTopic = (depthTopic[0] == '/') ? depthTopic : "/" + depthTopic;
        gzmsg << "DynamicProjector: Subscribing to depth data on " << fullDepthTopic << std::endl;
        node.Subscribe(fullDepthTopic + "/points", &DynamicProjector::OnNewDepthPointCloud, this);
        node.Subscribe(fullDepthTopic + "/camera_info", &DynamicProjector::OnDepthCameraInfo, this);
    }

    if (enhanceRgb){
    node.Subscribe("/color/image_raw", &DynamicProjector::OnNewColorImage, this);
    enhancedPub = node.Advertise<msgs::Image>("/color/enhanced");}

    // Use topic naming scheme from user request
    imagePub1 = node.Advertise<msgs::Image>(sensor1Topic + "/ir/image");

    if (mode == "stereo")
    {
        node.Subscribe(sensor2Topic, &DynamicProjector::OnNewImage2, this);
        node.Subscribe(sensor2Topic + "/image", &DynamicProjector::OnNewImage2, this);
        // node.Subscribe(sensor2Topic + "/points", &DynamicProjector::OnNewPointCloud2, this);
        node.Subscribe(sensor2Topic + "/camera_info", &DynamicProjector::OnCameraInfo2, this);
        imagePub2 = node.Advertise<msgs::Image>(sensor2Topic + "/ir/image");
    }

    GeneratePattern();
    initialized = true;
}

void DynamicProjector::OnNewDepthPointCloud(const msgs::PointCloudPacked &msg) 
{ 
    if (lastDepthCloud.width() == 0)
        gzmsg << "DynamicProjector: Received first depth point cloud" << std::endl;
    lastDepthCloud = msg; 
}

void DynamicProjector::OnDepthCameraInfo(const msgs::CameraInfo &msg)
{
    if (!depthInfoReceived && msg.has_intrinsics())
    {
        for (int i=0; i<9; ++i) this->depth_K[i] = msg.intrinsics().k(i);
        depthInfoReceived = true;
        gzmsg << "DynamicProjector: Received camera info for depth topic" << std::endl;
    }
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
    unsigned int width = msg.width();
    unsigned int height = msg.height();
    size_t totalPixels = static_cast<size_t>(width) * height;

    msgs::Image irMsg;
    irMsg.set_width(width);
    irMsg.set_height(height);
    irMsg.set_pixel_format_type(msgs::PixelFormatType::L_INT8);
    irMsg.set_step(width);
    irMsg.mutable_header()->CopyFrom(msg.header());
    
    std::string *imgData = irMsg.mutable_data();
    imgData->resize(totalPixels);
    unsigned char *dst = reinterpret_cast<unsigned char *>(imgData->data());

    // Physical IR Simulation Parameters (No noise as requested)
    const float ir_baseline_dim = 0.4f;   
    const float vignette_strength = 0.6f;  

    float cx = width / 2.0f;
    float cy = height / 2.0f;
    float maxDistSq = cx*cx + cy*cy;

    if (msg.pixel_format_type() == msgs::PixelFormatType::RGB_INT8)
    {
        const unsigned char *src = reinterpret_cast<const unsigned char *>(msg.data().data());
        for (size_t i = 0; i < totalPixels; ++i)
        {
            size_t y_idx = i / width;
            size_t x_idx = i % width;

            float Y = 0.299f * src[3*i] + 0.587f * src[3*i + 1] + 0.114f * src[3*i + 2];
            float val = Y * ir_baseline_dim;

            float dx = x_idx - cx;
            float dy = y_idx - cy;
            float distSq = dx*dx + dy*dy;
            float vignette = 1.0f - (vignette_strength * (distSq / maxDistSq));
            val *= vignette;

            dst[i] = static_cast<unsigned char>(std::clamp(val, 0.0f, 255.0f));
        }
    }
    else // L_INT8
    {
        const unsigned char *src = reinterpret_cast<const unsigned char *>(msg.data().data());
        for (size_t i = 0; i < totalPixels; ++i)
        {
            float val = static_cast<float>(src[i]) * ir_baseline_dim;
            dst[i] = static_cast<unsigned char>(std::clamp(val, 0.0f, 255.0f));
        }
    }

    OverlayDots(irMsg, cloud, isIr1, 128); 

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

    float baseLaserScale = (laserPower < 15) ? 0.0f : (static_cast<float>(laserPower - 15) / (360.0f - 15.0f));
    float ambienceNorm = static_cast<float>(ambienceScore) / 255.0f;
    float effectiveLaserScale = baseLaserScale * (1.0f - ambienceNorm);
    
    // Higher visibility parameters
    float dotAlphaBase = std::clamp(0.6f + effectiveLaserScale * 0.4f, 0.0f, 1.0f); 
    float dotBaseVal = 220.0f; 

    auto drawDot = [&](int u, int v, float dist, float geomFactor, const DotInfo &dot) {
        float baseRadius = std::clamp(6.0f / (dist + 1e-3f), 1.2f, 4.5f);
        float rx = baseRadius * (dot.shape == 1 ? 1.5f : (dot.shape == 2 ? 0.6f : 1.0f));
        float ry = baseRadius * (dot.shape == 1 ? 0.6f : (dot.shape == 2 ? 1.8f : 1.0f));

        float bg_center = static_cast<float>(data[v * width + u]);
        float albedo_proxy = std::clamp(0.5f + 0.5f * (bg_center / 128.0f), 0.3f, 1.0f);

        // Boosted intensity for visibility
        float intensity_falloff = std::clamp(geomFactor / (dist * 0.5f + 0.1f), 0.3f, 2.0f);
        float finalIntensity = dotBaseVal * albedo_proxy * intensity_falloff;

        for (int dy = -static_cast<int>(ry); dy <= static_cast<int>(ry); ++dy) {
            for (int dx = -static_cast<int>(rx); dx <= static_cast<int>(rx); ++dx) {
                float nxNorm = (float)dx / rx;
                float nyNorm = (float)dy / ry;
                if (dot.shape != 2 && (nxNorm * nxNorm + nyNorm * nyNorm) > 1.0f) continue;
                int nx = u + dx; int ny = v + dy;
                if (nx < 0 || nx >= (int)width || ny < 0 || ny >= (int)height) continue;

                float alpha = dotAlphaBase;
                if (dot.shape != 2) alpha *= (1.0f - (nxNorm * nxNorm + nyNorm * nyNorm));
                alpha = std::clamp(alpha, 0.0f, 1.0f);

                float bg = static_cast<float>(data[ny * width + nx]);
                // Additive light blending ensures dots are always brighter than background
                float out = bg + alpha * finalIntensity;
                data[ny * width + nx] = static_cast<unsigned char>(std::clamp(out, 0.0f, 255.0f));
            }
        }
    };

    float baseline = this->disparityOffset;
    math::Vector3d cam1_pos(0, baseline / 2.0, 0); 
    math::Vector3d cam2_pos(0, -baseline / 2.0, 0);

    auto findIntersection = [&](const msgs::PointCloudPacked& pc, float h, float v, float fx, float cx, float fy, float cy) -> std::tuple<math::Vector3d, math::Vector3d, bool>
    {
        int u = static_cast<int>(fx * std::tan(h) + cx);
        int v_px = static_cast<int>(fy * std::tan(v) + cy);
        if (u < 1 || u >= (int)pc.width() - 1 || v_px < 1 || v_px >= (int)pc.height() - 1) return std::make_tuple(math::Vector3d(), math::Vector3d(), false);

        auto getP = [&](int _u, int _v) {
            size_t idx = (_v * pc.width() + _u) * pc.point_step();
            return math::Vector3d(
                *reinterpret_cast<const float*>(&pc.data()[idx + pc.field(0).offset()]),
                *reinterpret_cast<const float*>(&pc.data()[idx + pc.field(1).offset()]),
                *reinterpret_cast<const float*>(&pc.data()[idx + pc.field(2).offset()])
            );
        };

        math::Vector3d p0 = getP(u, v_px);
        if(!std::isfinite(p0.X()) || p0.X() <= 0) return std::make_tuple(math::Vector3d(), math::Vector3d(), false);

        math::Vector3d p1 = getP(u + 1, v_px);
        math::Vector3d p2 = getP(u, v_px + 1);
        math::Vector3d n(1, 0, 0);
        if (std::isfinite(p1.X()) && std::isfinite(p2.X())) {
            n = (p1 - p0).Cross(p2 - p0);
            n.Normalize();
        }

        return {p0, n, true};
    };

    for (const auto &dot : this->dotPattern)
    {
        math::Vector3d p_world, normal;
        bool found = false;

        if (!depthTopic.empty() && lastDepthCloud.width() > 0 && depthInfoReceived) {
            auto [p, n, f] = findIntersection(lastDepthCloud, dot.h, dot.v, static_cast<float>(depth_K[0]), static_cast<float>(depth_K[2]), static_cast<float>(depth_K[4]), static_cast<float>(depth_K[5]));
            if (f) { p_world = p; normal = n; found = true; }
        } 
        
        if (!found && lastCloud1.width() > 0 && cam1InfoReceived) {
            auto [p, n, f] = findIntersection(lastCloud1, dot.h + baseline/2.0f, dot.v, static_cast<float>(cam1_K[0]), static_cast<float>(cam1_K[2]), static_cast<float>(cam1_K[4]), static_cast<float>(cam1_K[5]));
            if (f) { p_world = p + cam1_pos; normal = n; found = true; }
        }

        if (!found) continue;

        math::Vector3d ray_dir(1, -std::tan(dot.h), -std::tan(dot.v));
        ray_dir.Normalize();
        float cos_theta = std::abs(static_cast<float>(normal.Dot(-ray_dir)));

        const double* k_ir = isIr1 ? (cam1InfoReceived ? cam1_K.data() : (depthInfoReceived ? depth_K.data() : nullptr)) 
                                   : (cam2InfoReceived ? cam2_K.data() : (depthInfoReceived ? depth_K.data() : nullptr));
        if (!k_ir) continue;

        math::Vector3d p_cam = p_world - (isIr1 ? cam1_pos : cam2_pos);
        if (p_cam.X() <= 0) continue;

        float fx_ir = static_cast<float>(k_ir[0]), cx_ir = static_cast<float>(k_ir[2]), fy_ir = static_cast<float>(k_ir[4]), cy_ir = static_cast<float>(k_ir[5]);
        int u = static_cast<int>(cx_ir - fx_ir * (p_cam.Y() / p_cam.X()));
        int v = static_cast<int>(cy_ir - fy_ir * (p_cam.Z() / p_cam.X()));

        if (u >= 0 && u < (int)width && v >= 0 && v < (int)height)
            drawDot(u, v, static_cast<float>(p_cam.Length()), cos_theta, dot);
    }
}


void DynamicProjector::PostUpdate(const sim::UpdateInfo &, const sim::EntityComponentManager &) {}

GZ_ADD_PLUGIN(DynamicProjector,
              gz::sim::System,
              DynamicProjector::ISystemConfigure,
              DynamicProjector::ISystemPostUpdate)
