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

static double getDoubleFromBBox(const std::string& bbox_str, const std::string& key) {
  size_t pos = bbox_str.find("\"" + key + "\"");
  if (pos == std::string::npos) return 0.0;
  size_t colon = bbox_str.find(':', pos);
  size_t start = bbox_str.find_first_not_of(" \t\n", colon + 1);
  size_t end = bbox_str.find_first_of(",}\n]", start);
  std::string val = bbox_str.substr(start, end - start);
  val.erase(remove_if(val.begin(), val.end(), ::isspace), val.end());
  return std::stod(val);
}

bool loadDetectionsNormalized(const std::string& path,
                               NormalizedDetectionsOutput& out) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) return false;
  std::stringstream ss;
  ss << ifs.rdbuf();
  std::string json = ss.str();

  std::string res_obj = extractObject(json, "resolution");
  out.width  = getInt(res_obj, "width");
  out.height = getInt(res_obj, "height");

  std::string det_arr = extractArray(json, "detections");
  std::vector<std::string> objects = splitObjects(det_arr);

  out.detections.clear();
  for (const auto& obj : objects) {
    DetectionNormalized det;
    det.class_id   = getInt(obj, "class_id");
    det.confidence = getDouble(obj, "confidence");

    std::string bbox_str = extractObject(obj, "bbox");
    det.bbox.x_center = getDoubleFromBBox(bbox_str, "x_center");
    det.bbox.y_center = getDoubleFromBBox(bbox_str, "y_center");
    det.bbox.width    = getDoubleFromBBox(bbox_str, "width");
    det.bbox.height   = getDoubleFromBBox(bbox_str, "height");

    out.detections.push_back(det);
  }
  return true;
}

