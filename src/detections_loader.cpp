#include <fstream>
#include <sstream>
#include <algorithm>
#include "lidar_camera_projection/detections.hpp"

static double getDouble(const std::string& json, const std::string& key) {
  size_t pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return 0.0;
  size_t colon = json.find(':', pos);
  size_t start = json.find_first_not_of(" \t\n", colon + 1);
  size_t end = json.find_first_of(",}\n", start);
  std::string val = json.substr(start, end - start);
  val.erase(remove_if(val.begin(), val.end(), ::isspace), val.end());
  return std::stod(val);
}

static int getInt(const std::string& json, const std::string& key) {
  size_t pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return 0;
  size_t colon = json.find(':', pos);
  size_t start = json.find_first_not_of(" \t\n", colon + 1);
  size_t end = json.find_first_of(",}\n", start);
  std::string val = json.substr(start, end - start);
  val.erase(remove_if(val.begin(), val.end(), ::isspace), val.end());
  return std::stoi(val);
}

static std::string extractObject(const std::string& json, const std::string& key) {
  int brace_count = 0;
  size_t start = json.find("\"" + key + "\"");
  if (start == std::string::npos) return "";
  size_t pos = json.find('{', start);
  if (pos == std::string::npos) return "";
  std::string result;
  for (size_t i = pos; i < json.size(); ++i) {
    char c = json[i];
    result += c;
    if (c == '{') brace_count++;
    else if (c == '}') {
      brace_count--;
      if (brace_count == 0) return result;
    }
  }
  return "";
}

static std::string extractArray(const std::string& json, const std::string& key) {
  size_t start = json.find("\"" + key + "\"");
  if (start == std::string::npos) return "";
  size_t pos = json.find('[', start);
  if (pos == std::string::npos) return "";
  int brace_count = 0;
  std::string result;
  for (size_t i = pos; i < json.size(); ++i) {
    char c = json[i];
    result += c;
    if (c == '{') brace_count++;
    else if (c == '}') brace_count--;
    else if (c == ']' && brace_count == 0) return result;
  }
  return "";
}

static std::vector<std::string> splitObjects(const std::string& array_str) {
  std::vector<std::string> objects;
  int brace_count = 0;
  std::string current;
  for (size_t i = 0; i < array_str.size(); ++i) {
    char c = array_str[i];
    if (c == '{') {
      brace_count++;
      current += c;
    } else if (c == '}') {
      brace_count--;
      current += c;
      if (brace_count == 0) {
        objects.push_back(current);
        current.clear();
      }
    } else if (brace_count > 0) {
      current += c;
    }
  }
  return objects;
}

bool loadAndSaveDetections(const std::string& input_path,
                            const std::string& output_path,
                            int img_width,
                            int img_height) {
  std::ifstream ifs(input_path);
  if (!ifs.is_open()) return false;
  std::stringstream ss;
  ss << ifs.rdbuf();
  std::string json = ss.str();

  std::string res_obj = extractObject(json, "resolution");
  int width  = getInt(res_obj, "width");
  int height = getInt(res_obj, "height");
  if (width == 0 || height == 0) return false;

  std::string det_arr = extractArray(json, "detections");
  std::vector<std::string> objects = splitObjects(det_arr);

  std::vector<Detection> detections;
  for (const auto& obj : objects) {
    Detection det;
    det.class_id  = getInt(obj, "class_id");
    det.confidence = getDouble(obj, "confidence");

    std::string bbox_str = extractObject(obj, "bbox");
    double x_center = getDouble(bbox_str, "x_center");
    double y_center = getDouble(bbox_str, "y_center");
    double bw       = getDouble(bbox_str, "width");
    double bh       = getDouble(bbox_str, "height");

    det.bbox_pixel.xmin = static_cast<int>((x_center - bw * 0.5) * width);
    det.bbox_pixel.ymin = static_cast<int>((y_center - bh * 0.5) * height);
    det.bbox_pixel.xmax = static_cast<int>((x_center + bw * 0.5) * width);
    det.bbox_pixel.ymax = static_cast<int>((y_center + bh * 0.5) * height);
    det.bbox_pixel.computeCorners();

    detections.push_back(det);
  }

  // save as bbox_pixel 格式
  std::ofstream ofs(output_path);
  if (!ofs.is_open()) return false;
  ofs << "{\n";
  ofs << "  \"resolution\": {\n";
  ofs << "    \"width\": " << img_width << ",\n";
  ofs << "    \"height\": " << img_height << "\n";
  ofs << "  },\n";
  ofs << "  \"detections\": [\n";
  for (size_t i = 0; i < detections.size(); ++i) {
    const auto& d = detections[i];
    const auto& b = d.bbox_pixel;
    ofs << "    {\n";
    ofs << "      \"class_id\": " << d.class_id << ",\n";
    ofs << "      \"bbox_pixel\": {\n";
    ofs << "        \"xmin\": " << b.xmin << ", \"ymin\": " << b.ymin << ",\n";
    ofs << "        \"xmax\": " << b.xmax << ", \"ymax\": " << b.ymax << "\n";
    ofs << "      },\n";
    ofs << "      \"corners\": {\n";
    ofs << "        \"tl\": [" << b.tl_x << ", " << b.tl_y << "],\n";
    ofs << "        \"tr\": [" << b.tr_x << ", " << b.tr_y << "],\n";
    ofs << "        \"bl\": [" << b.bl_x << ", " << b.bl_y << "],\n";
    ofs << "        \"br\": [" << b.br_x << ", " << b.br_y << "]\n";
    ofs << "      },\n";
    ofs << "      \"confidence\": " << d.confidence << "\n";
    ofs << "    }" << (i + 1 == detections.size() ? "\n" : ",\n");
  }
  ofs << "  ]\n";
  ofs << "}\n";
  ofs.close();
  return true;
}
