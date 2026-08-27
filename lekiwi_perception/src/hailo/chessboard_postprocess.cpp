// Copyright 2026 LeKiwi Labs. All rights reserved.
#include "hailo/chessboard_postprocess.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <random>

namespace lekiwi_perception::hailo
{

struct CandidateInfo {
  int stride;
  int r;
  int c;
  float box[4];  // x1, y1, x2, y2 in 640x640 space
};

struct ChessboardCandidateResult {
  float score = 0.0f;
  float box[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // x1, y1, x2, y2 in 640x640 space
  std::vector<cv::Point2f> poly_pts;        // Normalized [0, 1] contour points
  cv::Mat mask_160;                         // 160x160 CV_32FC1 probability mask
  bool valid = false;
};

class YoloV8SegDecoder {
public:
  static inline float get_tensor_float_val(HailoTensorPtr t, int r, int c, int ch) {
    if (!t) return 0.0f;
    auto type = t->format().type;
    if (type == HailoTensorFormatType::HAILO_FORMAT_TYPE_FLOAT32) {
      const float * f = reinterpret_cast<const float *>(t->data());
      return f[(r * t->width() + c) * t->features() + ch];
    } else if (type == HailoTensorFormatType::HAILO_FORMAT_TYPE_UINT16) {
      return t->fix_scale(t->get_uint16(r, c, ch));
    } else {
      return t->fix_scale(t->get(r, c, ch));
    }
  }

  /**
   * Decode raw YOLOv8-segmentation tensors attached to HailoROI.
   */
  static bool decode(HailoROIPtr roi, ChessboardCandidateResult & out_result) {
    if (!roi) return false;
    std::vector<HailoTensorPtr> tensors = roi->get_tensors();
    if (tensors.empty()) return false;

    HailoTensorPtr proto_tensor = nullptr;
    std::map<int, HailoTensorPtr> cv2_tensors;  // DFL BBox
    std::map<int, HailoTensorPtr> cv3_tensors;  // Class scores
    std::map<int, HailoTensorPtr> cv4_tensors;  // Mask coeffs

    for (size_t idx = 0; idx < tensors.size(); ++idx) {
      const auto & t = tensors[idx];
      int h = t->height();
      int w = t->width();
      int c = t->features();

      if (h == 160 && w == 160 && c == 32) {
        proto_tensor = t;
      } else {
        int stride = 0;
        if (h == 80 && w == 80) stride = 8;
        else if (h == 40 && w == 40) stride = 16;
        else if (h == 20 && w == 20) stride = 32;

        if (stride > 0) {
          if (c == 64) cv2_tensors[stride] = t;
          else if (c == 32) cv4_tensors[stride] = t;
          else if (c <= 80) cv3_tensors[stride] = t;
        }
      }
    }

    if (!proto_tensor || cv3_tensors.empty()) return false;

    std::vector<cv::Rect2d> all_boxes_rect;
    std::vector<float> all_scores;
    std::vector<CandidateInfo> all_candidates;

    for (int stride : {8, 16, 32}) {
      if (cv3_tensors.find(stride) == cv3_tensors.end() ||
          cv2_tensors.find(stride) == cv2_tensors.end() ||
          cv4_tensors.find(stride) == cv4_tensors.end()) {
        continue;
      }

      auto t_cls = cv3_tensors[stride];
      auto t_dfl = cv2_tensors[stride];
      int h = t_cls->height();
      int w = t_cls->width();

      for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
          float raw_val = get_tensor_float_val(t_cls, r, c, 0);
          if (raw_val < -1.0986123f) continue;
          float score = 1.0f / (1.0f + std::exp(-raw_val));

          float dists[4] = {0.0f, 0.0f, 0.0f, 0.0f};
          for (int g = 0; g < 4; ++g) {
            float max_v = -1e9f;
            float vals[16];
            for (int k = 0; k < 16; ++k) {
              float val = get_tensor_float_val(t_dfl, r, c, g * 16 + k);
              vals[k] = val;
              if (val > max_v) max_v = val;
            }
            float sum_exp = 0.0f;
            float exp_vals[16];
            for (int k = 0; k < 16; ++k) {
              exp_vals[k] = std::exp(vals[k] - max_v);
              sum_exp += exp_vals[k];
            }
            for (int k = 0; k < 16; ++k) {
              dists[g] += k * (exp_vals[k] / sum_exp);
            }
          }

          float cx = (c + 0.5f) * stride;
          float cy = (r + 0.5f) * stride;
          float x1 = std::max(0.0f, std::min(640.0f, cx - dists[0] * stride));
          float y1 = std::max(0.0f, std::min(640.0f, cy - dists[1] * stride));
          float x2 = std::max(0.0f, std::min(640.0f, cx + dists[2] * stride));
          float y2 = std::max(0.0f, std::min(640.0f, cy + dists[3] * stride));

          float bw = x2 - x1;
          float bh = y2 - y1;

          if (bw >= 60.0f && bh >= 60.0f) {
            all_boxes_rect.push_back(cv::Rect2d(x1, y1, bw, bh));
            all_scores.push_back(score);
            all_candidates.push_back({stride, r, c, {x1, y1, x2, y2}});
          }
        }
      }
    }

    if (all_candidates.empty()) return false;

    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(all_boxes_rect, all_scores, 0.25f, 0.45f, nms_indices);
    if (nms_indices.empty()) return false;

    int best_nms_idx = nms_indices[0];
    float max_score = all_scores[best_nms_idx];
    for (int idx : nms_indices) {
      if (all_scores[idx] > max_score) {
        max_score = all_scores[idx];
        best_nms_idx = idx;
      }
    }

    const auto & best_cand = all_candidates[best_nms_idx];
    int best_stride = best_cand.stride;
    int best_y = best_cand.r;
    int best_x = best_cand.c;

    auto t_coeff = cv4_tensors[best_stride];
    std::vector<float> coeffs(32);
    for (int i = 0; i < 32; ++i) {
      coeffs[i] = get_tensor_float_val(t_coeff, best_y, best_x, i);
    }

    cv::Mat binary_mat = cv::Mat::zeros(160, 160, CV_8UC1);
    int pad = 4;  // Margin padding on 160x160 space to prevent clipping perspective trapezoid corners
    int px1 = std::max(0, std::min(160, static_cast<int>(best_cand.box[0] * (160.0f / 640.0f)) - pad));
    int py1 = std::max(0, std::min(160, static_cast<int>(best_cand.box[1] * (160.0f / 640.0f)) - pad));
    int px2 = std::max(0, std::min(160, static_cast<int>(best_cand.box[2] * (160.0f / 640.0f)) + pad));
    int py2 = std::max(0, std::min(160, static_cast<int>(best_cand.box[3] * (160.0f / 640.0f)) + pad));

    auto proto_type = proto_tensor->format().type;
    if (proto_type == HailoTensorFormatType::HAILO_FORMAT_TYPE_FLOAT32) {
      const float * proto_data = reinterpret_cast<const float *>(proto_tensor->data());
      for (int pr = py1; pr < py2; ++pr) {
        uint8_t * bin_row = binary_mat.ptr<uint8_t>(pr);
        const float * proto_pixel = proto_data + (pr * 160 + px1) * 32;
        for (int pc = px1; pc < px2; ++pc, proto_pixel += 32) {
          float dot = 0.0f;
          #pragma GCC unroll 32
          for (int k = 0; k < 32; ++k) {
            dot += coeffs[k] * proto_pixel[k];
          }
          if (dot > 0.0f) {
            bin_row[pc] = 255;
          }
        }
      }
    } else if (proto_type == HailoTensorFormatType::HAILO_FORMAT_TYPE_UINT8) {
      const uint8_t * proto_data = reinterpret_cast<const uint8_t *>(proto_tensor->data());
      float sum_coeffs = 0.0f;
      for (int k = 0; k < 32; ++k) sum_coeffs += coeffs[k];
      float bias_threshold = proto_tensor->quant_info().qp_zp * sum_coeffs;

      for (int pr = py1; pr < py2; ++pr) {
        uint8_t * bin_row = binary_mat.ptr<uint8_t>(pr);
        const uint8_t * proto_pixel = proto_data + (pr * 160 + px1) * 32;
        for (int pc = px1; pc < px2; ++pc, proto_pixel += 32) {
          float dot = 0.0f;
          #pragma GCC unroll 32
          for (int k = 0; k < 32; ++k) {
            dot += coeffs[k] * static_cast<float>(proto_pixel[k]);
          }
          if (dot > bias_threshold) {
            bin_row[pc] = 255;
          }
        }
      }
    } else if (proto_type == HailoTensorFormatType::HAILO_FORMAT_TYPE_UINT16) {
      const uint16_t * proto_data = reinterpret_cast<const uint16_t *>(proto_tensor->data());
      float sum_coeffs = 0.0f;
      for (int k = 0; k < 32; ++k) sum_coeffs += coeffs[k];
      float bias_threshold = proto_tensor->quant_info().qp_zp * sum_coeffs;

      for (int pr = py1; pr < py2; ++pr) {
        uint8_t * bin_row = binary_mat.ptr<uint8_t>(pr);
        const uint16_t * proto_pixel = proto_data + (pr * 160 + px1) * 32;
        for (int pc = px1; pc < px2; ++pc, proto_pixel += 32) {
          float dot = 0.0f;
          #pragma GCC unroll 32
          for (int k = 0; k < 32; ++k) {
            dot += coeffs[k] * static_cast<float>(proto_pixel[k]);
          }
          if (dot > bias_threshold) {
            bin_row[pc] = 255;
          }
        }
      }
    } else {
      for (int pr = py1; pr < py2; ++pr) {
        uint8_t * bin_row = binary_mat.ptr<uint8_t>(pr);
        for (int pc = px1; pc < px2; ++pc) {
          float dot = 0.0f;
          for (int k = 0; k < 32; ++k) {
            dot += coeffs[k] * get_tensor_float_val(proto_tensor, pr, pc, k);
          }
          if (dot > 0.0f) {
            bin_row[pc] = 255;
          }
        }
      }
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    float max_a = -1.0f;
    std::vector<cv::Point2f> poly_pts;

    for (const auto & cnt : contours) {
      float a = static_cast<float>(cv::contourArea(cnt));
      if (a > max_a && a >= 100.0f && cnt.size() >= 4) {
        max_a = a;
        poly_pts.clear();
        for (const auto & pt : cnt) {
          poly_pts.push_back(cv::Point2f(
            static_cast<float>(pt.x) / 160.0f,
            static_cast<float>(pt.y) / 160.0f
          ));
        }
      }
    }

    if (poly_pts.size() < 4) {
      if ((best_cand.box[2] - best_cand.box[0]) < 60.0f || (best_cand.box[3] - best_cand.box[1]) < 60.0f) {
        return false;
      }
      poly_pts = {
        cv::Point2f(best_cand.box[0] / 640.0f, best_cand.box[1] / 640.0f),
        cv::Point2f(best_cand.box[2] / 640.0f, best_cand.box[1] / 640.0f),
        cv::Point2f(best_cand.box[2] / 640.0f, best_cand.box[3] / 640.0f),
        cv::Point2f(best_cand.box[0] / 640.0f, best_cand.box[3] / 640.0f)
      };
    }

    out_result.score = max_score;
    out_result.box[0] = best_cand.box[0] / 640.0f;
    out_result.box[1] = best_cand.box[1] / 640.0f;
    out_result.box[2] = best_cand.box[2] / 640.0f;
    out_result.box[3] = best_cand.box[3] / 640.0f;
    out_result.poly_pts = poly_pts;
    out_result.mask_160 = binary_mat;
    out_result.valid = true;
    return true;
  }
};


float pointToSegmentDistance(const cv::Point2f & p, const cv::Point2f & a, const cv::Point2f & b) {
  cv::Point2f v = b - a;
  cv::Point2f w = p - a;
  float c1 = w.dot(v);
  if (c1 <= 0.0f) return cv::norm(p - a);

  float c2 = v.dot(v);
  if (c2 <= c1) return cv::norm(p - b);

  cv::Point2f proj = a + (c1 / c2) * v;
  return cv::norm(p - proj);
}

std::vector<cv::Point2f> sortCornersByPolarAngle(const std::vector<cv::Point2f> & corners) {
  if (corners.size() != 4) return corners;

  cv::Point2f centroid(0.0f, 0.0f);
  for (const auto & pt : corners) centroid += pt;
  centroid.x /= 4.0f;
  centroid.y /= 4.0f;

  struct PolarPoint {
    cv::Point2f pt;
    float angle;
  };
  std::vector<PolarPoint> polar_pts(4);

  for (size_t i = 0; i < 4; ++i) {
    float angle = std::atan2(corners[i].y - centroid.y, corners[i].x - centroid.x);
    polar_pts[i] = {corners[i], angle};
  }

  std::sort(polar_pts.begin(), polar_pts.end(), [](const PolarPoint & a, const PolarPoint & b) {
    return a.angle < b.angle;
  });

  std::vector<cv::Point2f> sorted_corners(4);
  for (size_t i = 0; i < 4; ++i) sorted_corners[i] = polar_pts[i].pt;

  size_t tl_idx = 0;
  float min_sum = sorted_corners[0].x + sorted_corners[0].y;
  for (size_t i = 1; i < 4; ++i) {
    float sum = sorted_corners[i].x + sorted_corners[i].y;
    if (sum < min_sum) {
      min_sum = sum;
      tl_idx = i;
    }
  }
  if (tl_idx != 0) {
    std::rotate(sorted_corners.begin(), sorted_corners.begin() + tl_idx, sorted_corners.end());
  }

  return sorted_corners;
}

std::vector<cv::Point2f> extractInitial4Corners(const std::vector<cv::Point2f> & polygon_points) {
  if (polygon_points.size() < 4) {
    return polygon_points;
  }

  std::vector<cv::Point2f> hull;
  cv::convexHull(polygon_points, hull);
  double peri = cv::arcLength(hull, true);

  for (int step = 0; step < 10; ++step) {
    float eps_factor = 0.06f - (step * (0.06f - 0.01f) / 9.0f);
    std::vector<cv::Point2f> approx;
    cv::approxPolyDP(hull, approx, eps_factor * peri, true);
    if (approx.size() == 4 && cv::isContourConvex(approx)) {
      return sortCornersByPolarAngle(approx);
    }
  }

  cv::Point2f centroid(0.0f, 0.0f);
  for (const auto & pt : hull) centroid += pt;
  centroid.x /= static_cast<float>(hull.size());
  centroid.y /= static_cast<float>(hull.size());

  std::vector<float> target_angles = {
    -static_cast<float>(M_PI), -static_cast<float>(M_PI) / 2.0f, 0.0f, static_cast<float>(M_PI) / 2.0f
  };
  std::vector<cv::Point2f> extreme_pts;

  for (float target : target_angles) {
    float min_diff = 1e9f;
    cv::Point2f best_pt = hull[0];
    for (const auto & pt : hull) {
      float angle = std::atan2(pt.y - centroid.y, pt.x - centroid.x);
      float diff = std::abs(angle - target);
      diff = std::min(diff, static_cast<float>(2.0 * M_PI) - diff);
      if (diff < min_diff) {
        min_diff = diff;
        best_pt = pt;
      }
    }
    extreme_pts.push_back(best_pt);
  }
  return sortCornersByPolarAngle(extreme_pts);
}

bool bestFitLineRansac(
  const std::vector<cv::Point2f> & points, cv::Vec4f & out_line,
  std::vector<cv::Point2f> & out_inliers, float threshold, int max_iters)
{
  size_t num_points = points.size();
  if (num_points < 2) return false;

  size_t best_inlier_count = 0;
  size_t best_idx1 = 0;
  size_t best_idx2 = 1;

  std::mt19937 rng(42);
  std::uniform_int_distribution<size_t> dist(0, num_points - 1);
  size_t early_stop_thresh = static_cast<size_t>(num_points * 0.95f);
  float min_sample_dist = 0.03f;  // Require minimum distance between sample points in [0, 1] space

  for (int iter = 0; iter < max_iters; ++iter) {
    size_t idx1 = dist(rng);
    size_t idx2 = dist(rng);
    if (idx1 == idx2) continue;

    float dx = points[idx2].x - points[idx1].x;
    float dy = points[idx2].y - points[idx1].y;
    float len_sq = dx * dx + dy * dy;
    if (len_sq < (min_sample_dist * min_sample_dist)) continue;

    float inv_len = 1.0f / std::sqrt(len_sq);
    float nx = -dy * inv_len;
    float ny = dx * inv_len;
    float c = -(nx * points[idx1].x + ny * points[idx1].y);

    size_t inlier_count = 0;
    for (const auto & p : points) {
      float d = std::abs(nx * p.x + ny * p.y + c);
      if (d < threshold) {
        inlier_count++;
      }
    }

    if (inlier_count > best_inlier_count) {
      best_inlier_count = inlier_count;
      best_idx1 = idx1;
      best_idx2 = idx2;
      if (best_inlier_count >= early_stop_thresh) {
        break;
      }
    }
  }

  if (best_inlier_count >= 2) {
    float dx = points[best_idx2].x - points[best_idx1].x;
    float dy = points[best_idx2].y - points[best_idx1].y;
    float inv_len = 1.0f / std::sqrt(dx * dx + dy * dy);
    float nx = -dy * inv_len;
    float ny = dx * inv_len;
    float c = -(nx * points[best_idx1].x + ny * points[best_idx1].y);

    out_inliers.clear();
    out_inliers.reserve(best_inlier_count);
    for (const auto & p : points) {
      float d = std::abs(nx * p.x + ny * p.y + c);
      if (d < threshold) {
        out_inliers.push_back(p);
      }
    }

    cv::fitLine(out_inliers, out_line, cv::DIST_L2, 0, 0.01, 0.01);
    return true;
  }
  return false;
}

std::pair<std::vector<cv::Vec4f>, std::vector<cv::Point2f>> extract4EdgeLinesRansac(
  const std::vector<cv::Point2f> & polygon_points, float threshold, int max_iters)
{
  std::vector<cv::Point2f> init_corners = extractInitial4Corners(polygon_points);
  std::vector<std::pair<cv::Point2f, cv::Point2f>> segments = {
    {init_corners[0], init_corners[1]},  // Top edge: C0 -> C1
    {init_corners[1], init_corners[2]},  // Right edge: C1 -> C2
    {init_corners[2], init_corners[3]},  // Bottom edge: C2 -> C3
    {init_corners[3], init_corners[0]}   // Left edge: C3 -> C0
  };

  // Robust geometric distance-based clustering: independent of contour winding order (CW/CCW)
  std::vector<std::vector<cv::Point2f>> edge_clusters(4);
  for (const auto & pt : polygon_points) {
    float min_d = 1e9f;
    int best_edge = 0;
    for (int k = 0; k < 4; ++k) {
      float d = pointToSegmentDistance(pt, segments[k].first, segments[k].second);
      if (d < min_d) {
        min_d = d;
        best_edge = k;
      }
    }
    edge_clusters[best_edge].push_back(pt);
  }

  std::vector<cv::Vec4f> lines(4);
  for (size_t k = 0; k < 4; ++k) {
    cv::Vec4f line;
    std::vector<cv::Point2f> inliers;
    bool ok = bestFitLineRansac(edge_clusters[k], line, inliers, threshold, max_iters);
    if (!ok) {
      std::vector<cv::Point2f> seg = {segments[k].first, segments[k].second};
      cv::fitLine(seg, line, cv::DIST_L2, 0, 0.01, 0.01);
    }
    lines[k] = line;
  }
  return {lines, init_corners};
}

bool lineIntersection(const cv::Vec4f & line1, const cv::Vec4f & line2, cv::Point2f & intersection) {
  float vx1 = line1[0], vy1 = line1[1], x01 = line1[2], y01 = line1[3];
  float vx2 = line2[0], vy2 = line2[1], x02 = line2[2], y02 = line2[3];

  float det = vx1 * (-vy2) - vy1 * (-vx2);
  if (std::abs(det) < 1e-6f) return false;

  float b0 = x02 - x01;
  float b1 = y02 - y01;
  float t0 = (b0 * (-vy2) - b1 * (-vx2)) / det;

  intersection.x = x01 + t0 * vx1;
  intersection.y = y01 + t0 * vy1;

  if (intersection.x < -0.15f || intersection.x > 1.15f ||
      intersection.y < -0.15f || intersection.y > 1.15f) {
    return false;
  }
  return true;
}

bool extractChessboardPerspective(
  const std::vector<cv::Point2f> & polygon_points,
  cv::Mat & out_M, cv::Mat & out_M_inv,
  std::vector<std::vector<cv::Point2f>> & out_squares,
  std::vector<cv::Point2f> & out_corners,
  float threshold, int max_iters)
{
  if (polygon_points.size() < 4) return false;

  auto [lines, init_corners] = extract4EdgeLinesRansac(polygon_points, threshold, max_iters);

  cv::Point2f c0, c1, c2, c3;
  if (!lineIntersection(lines[3], lines[0], c0)) c0 = init_corners[0];
  if (!lineIntersection(lines[0], lines[1], c1)) c1 = init_corners[1];
  if (!lineIntersection(lines[1], lines[2], c2)) c2 = init_corners[2];
  if (!lineIntersection(lines[2], lines[3], c3)) c3 = init_corners[3];

  std::vector<cv::Point2f> corners_cw = sortCornersByPolarAngle({c0, c1, c2, c3});
  out_corners = corners_cw;

  std::vector<cv::Point2f> dst_pts = {
    cv::Point2f(0.0f, 0.0f),
    cv::Point2f(8.0f, 0.0f),
    cv::Point2f(8.0f, 8.0f),
    cv::Point2f(0.0f, 8.0f)
  };

  out_M = cv::getPerspectiveTransform(out_corners, dst_pts);
  cv::invert(out_M, out_M_inv);

  std::vector<cv::Point2f> warped_sq_pts;
  warped_sq_pts.reserve(64 * 4);

  for (int i = 7; i >= 0; --i) {
    for (int j = 0; j < 8; ++j) {
      float col_f = static_cast<float>(j);
      float row_f = static_cast<float>(i);
      warped_sq_pts.push_back(cv::Point2f(col_f, row_f));
      warped_sq_pts.push_back(cv::Point2f(col_f + 1.0f, row_f));
      warped_sq_pts.push_back(cv::Point2f(col_f + 1.0f, row_f + 1.0f));
      warped_sq_pts.push_back(cv::Point2f(col_f, row_f + 1.0f));
    }
  }

  std::vector<cv::Point2f> orig_sq_pts;
  cv::perspectiveTransform(warped_sq_pts, orig_sq_pts, out_M_inv);

  out_squares.clear();
  out_squares.reserve(64);
  for (size_t idx = 0; idx < 64; ++idx) {
    std::vector<cv::Point2f> sq = {
      orig_sq_pts[idx * 4 + 0], orig_sq_pts[idx * 4 + 1],
      orig_sq_pts[idx * 4 + 2], orig_sq_pts[idx * 4 + 3]
    };
    out_squares.push_back(sq);
  }
  return true;
}

}  // namespace lekiwi_perception::hailo

void ChessboardPostprocess::add_cached_result(HailoROIPtr roi) const
{
  roi->add_object(std::make_shared<HailoMatrix>(cached_mat_data_, 3, 3));
  roi->add_object(std::make_shared<HailoMatrix>(cached_grid_data_, 81, 2));
}

void ChessboardPostprocess::filter(HailoROIPtr roi)
{
  if (!roi) {
    return;
  }

  if (has_cache_) {
    ++frame_idx_;
    if ((frame_idx_ % kFullEvaluationInterval) != 0) {
      add_cached_result(roi);
      return;
    }
  }

  lekiwi_perception::hailo::ChessboardCandidateResult candidate;
  if (!lekiwi_perception::hailo::YoloV8SegDecoder::decode(roi, candidate) || candidate.poly_pts.size() < 4U) {
    if (has_cache_) {
      add_cached_result(roi);
    }
    return;
  }

  cv::Mat matrix;
  cv::Mat inverse_matrix;
  std::vector<std::vector<cv::Point2f>> squares;
  std::vector<cv::Point2f> corners;
  if (!lekiwi_perception::hailo::extractChessboardPerspective(
      candidate.poly_pts, matrix, inverse_matrix, squares, corners,
      kRansacThreshold, kMaxIterations))
  {
    if (has_cache_) {
      add_cached_result(roi);
    }
    return;
  }

  std::vector<float> mat_data(9U);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      mat_data[static_cast<std::size_t>(row * 3 + column)] =
        static_cast<float>(matrix.at<double>(row, column));
    }
  }
  roi->add_object(std::make_shared<HailoMatrix>(mat_data, 3, 3));

  std::vector<cv::Point2f> canonical_grid;
  canonical_grid.reserve(81U);
  for (int row = 0; row <= 8; ++row) {
    for (int column = 0; column <= 8; ++column) {
      canonical_grid.emplace_back(static_cast<float>(column), static_cast<float>(row));
    }
  }

  std::vector<cv::Point2f> original_grid;
  cv::perspectiveTransform(canonical_grid, original_grid, inverse_matrix);
  std::vector<float> grid_data;
  grid_data.reserve(162U);
  for (const auto & point : original_grid) {
    grid_data.push_back(point.x);
    grid_data.push_back(point.y);
  }
  roi->add_object(std::make_shared<HailoMatrix>(grid_data, 81, 2));

  cached_mat_data_ = std::move(mat_data);
  cached_grid_data_ = std::move(grid_data);
  has_cache_ = true;
  frame_idx_ = 0;
}

extern "C" {

ChessboardPostprocess * init(const std::string &, const std::string &)
{
  return new ChessboardPostprocess();
}

void free_resources(void * params_void_ptr)
{
  delete static_cast<ChessboardPostprocess *>(params_void_ptr);
}

void filter_chessboard(HailoROIPtr roi, void * params_void_ptr)
{
  auto * postprocess = static_cast<ChessboardPostprocess *>(params_void_ptr);
  if (postprocess) {
    postprocess->filter(roi);
  }
}

void filter(HailoROIPtr roi, void * params_void_ptr)
{
  filter_chessboard(roi, params_void_ptr);
}

}  // extern "C"
