#pragma once

#include <math.h>

#include <vector>

#include <geometry_msgs/msg/point.hpp>

// LANELET2 Header
#include <lanelet2_core/primitives/Lanelet.h>
// #include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_core/primitives/Point.h>
// #include <lanelet2_core/LaneletMap.h>
// #include <lanelet2_core/Forward.h>
// #include <lanelet2_core/primitives/Area.h>
// #include <lanelet2_core/geometry/Area.h>
// #include <lanelet2_core/geometry/Point.h>
// #include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
// #include <lanelet2_core/geometry/LaneletMap.h>
// #include <lanelet2_core/geometry/Polygon.h>
// #include <lanelet2_core/primitives/LaneletOrArea.h>
// #include <lanelet2_core/primitives/LaneletSequence.h>
// #include <lanelet2_core/primitives/BasicRegulatoryElements.h>
// #include <lanelet2_core/utility/Units.h>

// #include <lanelet2_io/Io.h>

// #include <lanelet2_projection/UTM.h>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
// #include <lanelet2_routing/RouteElement.h>
// #include <lanelet2_routing/LaneletPath.h>
// #include <lanelet2_routing/Forward.h>

// #include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

using namespace lanelet;
using namespace lanelet::traffic_rules;
using LaneletRouteSet = std::vector<ConstLanelets>;

struct PathElement {
  int64_t ll_id;
  bool inverted;
};

inline bool operator==(const PathElement &lhs, const PathElement &rhs) {
  return lhs.ll_id == rhs.ll_id && lhs.inverted == rhs.inverted;
};
inline bool operator!=(const PathElement &lhs, const PathElement &rhs) { return !(lhs == rhs); };

using Path = std::vector<PathElement>;

struct PointWithTime {
  float x;
  float y;
  float t;
};

using LineWithTime = std::vector<PointWithTime>;

class Lanelet2Utilities {
 public:
  /**
   * @brief returns the rough heading of the given lanelet at the given length
   * @param laneletLine - ConstLineString3d
   * @param length, length at which to look at - double
   * @return double, angle of Lanelet Line relative to UTM in rad
   */
  static double getLaneletLineHeading(const ConstLineString3d &laneletLine, const double &length) {
    return getLaneletLineHeading(utils::to2D(laneletLine).basicLineString(), length);
  }

  static double getLaneletLineHeading(const ConstLineString2d &laneletLine, const double &length) {
    return getLaneletLineHeading(laneletLine.basicLineString(), length);
  }

  static double getLaneletLineHeading(const BasicLineString2d &laneletLine, const double &length) {
    double total_length = 0;
    BasicPoint2d point_x_y_before =
        fromArcCoordinates_fast(laneletLine, std::max(0.0, length - 0.25), 0., total_length);
    if (total_length > 0) {
      point_x_y_before = fromArcCoordinates_fast(laneletLine, std::max(0.0, total_length - 0.25), 0.);
    }
    BasicPoint2d point_x_y_after = fromArcCoordinates_fast(laneletLine, length + 0.25, 0.);
    double lanelet_heading =
        atan2(point_x_y_after.y() - point_x_y_before.y(), point_x_y_after.x() - point_x_y_before.x());

    // Limit to 2PI
    while (lanelet_heading > M_PI * 2) {
      lanelet_heading -= M_PI * 2;
    }
    while (lanelet_heading < 0.) {
      lanelet_heading += M_PI * 2;
    }

    return lanelet_heading;
  }

  /**
   * @brief sorts a vector of lanelets according to their probability for being the correct matched lanelet
   * @param point, x/y point - BasicPoint2d
   * @param lanelets, vector of lanelets produced for example by geometry::findNearest() - std::vector<std::pair<double, Lanelet>>
   * @param heading(optional), optional heading of object or ego vehicle realtive to UTM in rad - double
   * @param trafficRules(optional), optional lanelet-traffic-rules-object which defines which lanelets are passable by which participant - traffic_rules::TrafficRulesPtr
   * @param last_lanelet_id(optional), optional last_lanelet_id for better matching over time - long int
   * @return nothing, but input vector of lanelets is sorted
   */
  static std::vector<double> laneletSorting(const BasicPoint2d &point,
                                            std::vector<std::pair<double, ConstLanelet>> &lanelets,
                                            boost::optional<const float &> heading,
                                            boost::optional<const traffic_rules::TrafficRulesPtr &> trafficRules,
                                            boost::optional<const int64_t &> last_lanelet_id) {
    // lanelets[i].first = distance to lanelet
    // lanelets[i].second = lanelet
    std::vector<double> costs;
    if (lanelets.empty()) {
      return costs;
    }

    struct lanelet_n_cost {
      std::pair<double, ConstLanelet> lanelet;
      double lanelet_cost;
    };

    std::vector<lanelet_n_cost> lanelets_n_costs;

    for (uint i = 0; i < lanelets.size(); i++) {
      const ArcCoordinates arccordinates_point = geometry::toArcCoordinates(lanelets[i].second.centerline2d(), point);
      const bool is_inside = lanelet::geometry::inside(lanelets[i].second, point);

      double cost_for_this_lanelet =
          (is_inside ? 0.5 : 2.0) * arccordinates_point.distance * arccordinates_point.distance;

      if (!!heading) {
        double diff_heading;  // rad

        diff_heading =
            std::abs(*heading - getLaneletLineHeading(lanelets[i].second.centerline(), arccordinates_point.length));

        if (diff_heading > M_PI) {
          diff_heading = M_PI * 2.0 - diff_heading;
        }

        if (!!trafficRules && !(*trafficRules)->isOneWay(lanelets[i].second) && diff_heading > M_PI / 2.0) {
          diff_heading = M_PI - diff_heading;
        }

        cost_for_this_lanelet += 10.0 * diff_heading * diff_heading;
      }

      if (!!trafficRules) {
        if (!(*trafficRules)->canPass(lanelets[i].second)) {
          cost_for_this_lanelet *= 2;
        }
      }

      if (!!last_lanelet_id) {
        if (*last_lanelet_id != lanelets[i].second.id()) {
          cost_for_this_lanelet *= 1.2;
        }
      }

      lanelets_n_costs.push_back({lanelets[i], cost_for_this_lanelet});
    }

    std::sort(lanelets_n_costs.begin(), lanelets_n_costs.end(),
              [](const auto &i, const auto &j) { return i.lanelet_cost < j.lanelet_cost; });

    for (uint i = 0; i < lanelets_n_costs.size(); i++) {
      lanelets.at(i) = lanelets_n_costs[i].lanelet;
      costs.push_back(lanelets_n_costs[i].lanelet_cost);
    }
    return costs;
  }

  static std::vector<geometry_msgs::msg::Point> convertLaneletLine2Linestring(const BasicLineString3d &ll_line) {
    std::vector<geometry_msgs::msg::Point> points;
    for (auto &p : ll_line) {
      geometry_msgs::msg::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      points.push_back(point);
    }
    return points;
  }

  static std::vector<geometry_msgs::msg::Point> convertLaneletLine2Linestring(const BasicLineString2d &ll_line) {
    std::vector<geometry_msgs::msg::Point> points;
    for (auto &p : ll_line) {
      geometry_msgs::msg::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = 0.0;
      points.push_back(point);
    }
    return points;
  }

  static BasicPoint2d fromArcCoordinates_fast(const BasicLineString2d &line, const double &length,
                                              const double &distance, boost::optional<double &> line_length = {}) {
    if (line.size() == 1) {
      return line.at(0);
    }

    double cur_len = 0.0;
    double cur_cum_len = 0.0;
    double remaining_dis = 0.0;
    size_t start_idx = 0;
    size_t end_idx = 0;
    bool is_s_bigger_length = false;

    for (size_t i = 0; i < line.size() - 1; i++) {
      cur_len = boost::geometry::distance(line.at(i), line.at(i + 1));
      cur_cum_len += cur_len;

      if (cur_cum_len > length) {
        remaining_dis = length - (cur_cum_len - cur_len);
        start_idx = i;
        end_idx = i + 1;
        break;
      }
    }

    // arc_length > line length
    if (end_idx == 0) {
      end_idx = line.size() - 1;
      start_idx = end_idx - 1;
      is_s_bigger_length = true;
      remaining_dis = boost::geometry::distance(line.at(start_idx), line.at(end_idx));

      if (!!line_length) {
        *line_length = cur_cum_len;
      }
    }

    // Very small line: generating point randomly around first point of line
    if (is_s_bigger_length && cur_cum_len < 0.05) {
      return line.at(0);
    }

    const double dx = cur_len == 0.0 ? 0 : (line.at(start_idx).x() - line.at(end_idx).x()) / cur_len;
    const double dy = cur_len == 0.0 ? 0 : (line.at(start_idx).y() - line.at(end_idx).y()) / cur_len;

    const double p_x = line.at(start_idx).x() - dx * remaining_dis;
    const double p_y = line.at(start_idx).y() - dy * remaining_dis;

    return BasicPoint2d(p_x + dy * distance, p_y - dx * distance);
  }

  static BasicPoint2d fromArcCoordinates_fast(const ConstLineString2d &line, const double &length,
                                              const double &distance) {
    return fromArcCoordinates_fast(line.basicLineString(), length, distance);
  }

  static BasicPoint2d fromArcCoordinates_fast(const ConstLineString3d &line, const double &length,
                                              const double &distance) {
    return fromArcCoordinates_fast(utils::to2D(line).basicLineString(), length, distance);
  }

  static BasicPoint2d fromArcCoordinates_fast(const BasicLineString2d &line, const ArcCoordinates &arc) {
    return fromArcCoordinates_fast(line, arc.length, arc.distance);
  }

  static BasicPoint2d fromArcCoordinates_fast(const ConstLineString2d &line, const ArcCoordinates &arc) {
    return fromArcCoordinates_fast(line.basicLineString(), arc.length, arc.distance);
  }

  static BasicPoint2d fromArcCoordinates_fast(const ConstLineString3d &line, const ArcCoordinates &arc) {
    return fromArcCoordinates_fast(utils::to2D(line).basicLineString(), arc.length, arc.distance);
  }

  static void addBoundarySegment(std::pair<BasicLineString2d, BasicLineString2d> &boundaries,
                                 const std::pair<BasicLineString2d, BasicLineString2d> &segments,
                                 const BasicLineString2d &centerline) {
    // Left
    {
      const BasicPoint2d boundaryEnd = getLastBoundaryPoint(segments.first, centerline, 10.0);
      BasicLineString2d line1, line2;
      splitLinestring(segments.first, boundaryEnd, line1, line2);
      boundaries.first.insert(boundaries.first.end(), line1.begin(), line1.end());
    }

    // Right
    {
      const BasicPoint2d boundaryEnd = getLastBoundaryPoint(segments.second, centerline, -10.0);
      BasicLineString2d line1, line2;
      splitLinestring(segments.second, boundaryEnd, line1, line2);
      boundaries.second.insert(boundaries.second.end(), line1.begin(), line1.end());
    }
  }

  static void delFromBoundarySegment(BasicLineString2d &left_segment, BasicLineString2d &right_segment,
                                     const BasicLineString2d &centerline) {
    // Left
    {
      const BasicPoint2d boundaryEnd = getLastBoundaryPoint(left_segment, centerline, 10.0);
      BasicLineString2d line1, line2;
      splitLinestring(left_segment, boundaryEnd, line1, line2);
      left_segment = line2;
    }

    // Right
    {
      const BasicPoint2d boundaryEnd = getLastBoundaryPoint(right_segment, centerline, -10.0);
      BasicLineString2d line1, line2;
      splitLinestring(right_segment, boundaryEnd, line1, line2);
      right_segment = line2;
    }
  }

  static BasicPoint2d getLastBoundaryPoint(const BasicLineString2d &boundary_segment,
                                           const BasicLineString2d &centerline, const double &max_distance) {
    const BasicPoint2d test_p =
        geometry::internal::lateralShiftPointAtIndex(centerline, centerline.size() - 1, max_distance);
    const BasicLineString2d test_line({centerline.back(), test_p});
    BasicPoints2d interpoints;
    boost::geometry::intersection(boundary_segment, test_line, interpoints);

    // Sort according to distance
    std::vector<std::pair<double, BasicPoint2d>> all_interpoints;
    for (const BasicPoint2d &poi : interpoints) {
      all_interpoints.emplace_back(geometry::distance(centerline.back(), poi), poi);
    }
    std::sort(all_interpoints.begin(), all_interpoints.end(),
              [](auto const &t1, auto const &t2) { return t1.first < t2.first; });

    return interpoints.size() ? all_interpoints.front().second : BasicPoint2d(boundary_segment.back());
  }

  static void splitLinestring(const BasicLineString2d &ls, const BasicPoint2d &pt, BasicLineString2d &line1,
                              BasicLineString2d &line2) {
    const ArcCoordinates arc_pt = geometry::toArcCoordinates(ls, pt);
    BasicPoint2d nearestPoint = geometry::nearestPointAtDistance(ls, arc_pt.length);
    const ArcCoordinates arc_nearest_pt = geometry::toArcCoordinates(ls, nearestPoint);
    auto idx = std::find(ls.begin(), ls.end(), nearestPoint);
    if (arc_pt.length < arc_nearest_pt.length && idx != ls.begin()) idx--;

    line1.clear();
    line1.insert(line1.begin(), ls.begin(), idx);
    line1.push_back(pt);

    line2.clear();
    line2.push_back(pt);
    line2.insert(line2.begin() + 1, idx + 1, ls.end());
  }

  static BasicLineString2d smoothByQuadraticBezierCurve(const BasicLineString2d &input_line, uint smooth_factor) {
    BasicLineString2d output_line;
    output_line.push_back(input_line.at(0));
    // quadratic bezier curve smoothing
    for (uint p0_idx = 0; p0_idx < input_line.size() - smooth_factor;
         p0_idx++) {  // https://stackoverflow.com/a/34750777
      // https://en.wikipedia.org/wiki/B%C3%A9zier_curve
      // https://stackoverflow.com/questions/5634460/quadratic-b%C3%A9zier-curve-calculate-points
      uint p1_idx = std::min(p0_idx + smooth_factor, uint(input_line.size()) - 1);
      uint p2_idx = std::min(p1_idx + smooth_factor, uint(input_line.size()) - 1);

      const BasicPoint2d &p0 = input_line.at(p0_idx);
      const BasicPoint2d &p1 = input_line.at(p1_idx);
      const BasicPoint2d &p2 = input_line.at(p2_idx);
      if (p0 == p1) {
        continue;
      }
      if (p1 == p2) {
        continue;
      }
      if (p0 == p2) {
        continue;
      }

      double t = 0.0;
      double len = 0.0;
      for (uint pp = p0_idx; pp < p2_idx; pp++) {
        if (pp == p1_idx) {
          t = len;
        }
        len += geometry::distance(input_line.at(pp), input_line.at(pp + 1));
      }
      t = t / len;

      double x = (1 - t) * (1 - t) * p0.x() + 2 * (1 - t) * t * p1.x() + t * t * p2.x();
      double y = (1 - t) * (1 - t) * p0.y() + 2 * (1 - t) * t * p1.y() + t * t * p2.y();

      output_line.push_back(BasicPoint2d(x, y));

      // double x1 = (p1.x() - p0.x()) * t + p0.x();
      // double y1 = (p1.y() - p0.y()) * t + p0.y();
      // double x2 = (p2.x() - p1.x()) * t + p1.x();
      // double y2 = (p2.y() - p1.y()) * t + p1.y();
      // output_line.push_back(BasicPoint2d((x2 - x1) * t + x1, (y2 - y1) * t + y1));
    }
    return output_line;
  }

  // Line conversions
  static void internalLine2llLine(BasicLineString2d &lanelet_line, const LineWithTime &line) {
    lanelet_line.clear();
    lanelet_line.reserve(line.size());

    for (const auto &point : line) {
      lanelet_line.push_back(BasicPoint2d(point.x, point.y));
    }
  }

  static void llLine2internalLine(const BasicLineString3d &lanelet_line, LineWithTime &line) {
    line.clear();
    line.reserve(lanelet_line.size());

    for (auto &lanelet_point : lanelet_line) {
      PointWithTime point;
      point.x = lanelet_point.x();
      point.y = lanelet_point.y();
      point.t = lanelet_point.z();

      line.push_back(point);
    }
  }

  // Path conversions
  static ConstLanelets internalPath2llPath(const Path &ll_id_vec, const LaneletMapConstPtr &ll_map) {
    ConstLanelets ll_route;
    ll_route.reserve(ll_id_vec.size());

    for (size_t i = 0; i < ll_id_vec.size(); ++i) {
      if (ll_id_vec[i].inverted) {
        ll_route.push_back(ll_map->laneletLayer.get(ll_id_vec[i].ll_id).invert());
      } else {
        ll_route.push_back(ll_map->laneletLayer.get(ll_id_vec[i].ll_id));
      }
    }
    return ll_route;
  }

  static Path llPath2internalPath(const ConstLanelets &ll_route) {
    Path ll_id_vec;
    ll_id_vec.reserve(ll_route.size());

    for (const auto &ll_id : ll_route) {
      ll_id_vec.push_back({ll_id.id(), ll_id.inverted()});
    }

    return ll_id_vec;
  }

  // Path to line conversions
  static LineWithTime llPath2internalLine(const ConstLanelets &ll_route) {
    LineWithTime line;
    line.reserve(ll_route.size());

    for (const auto &ll : ll_route) {
      for (const auto &p : ll.centerline()) {
        PointWithTime point;
        point.x = p.x();
        point.y = p.y();
        point.t = 0.0;
        line.push_back(point);
      }
    }

    return line;
  }

  static BasicLineString2d llPath2llLineTimeBased(const ConstLanelets &ll_path, const BasicPoint2d &cur_pos,
                                                  const double &vel, const double &t_ref, const double &t_max,
                                                  const double &dt,
                                                  boost::optional<const BasicPoint2d &> target_point) {
    BasicLineString2d path_line;

    std::vector<BasicLineString2d> segments;
    for (size_t l = 0; l < ll_path.size(); l++) {
      if (l < ll_path.size() - 1 &&
          (geometry::rightOf(ll_path.at(l), ll_path.at(l + 1)) || geometry::leftOf(ll_path.at(l), ll_path.at(l + 1)))) {
        segments.push_back(ll_path.at(l + 1).centerline2d().basicLineString());
        l++;
        continue;
      }

      if (l == 0) {
        segments.push_back(ll_path.at(0).centerline2d().basicLineString());
      } else {
        BasicLineString2d line = ll_path.at(l).centerline2d().basicLineString();
        segments.back().insert(segments.back().end(), line.begin() + 1, line.end());
      }
    }

    double target_s = std::numeric_limits<double>::max();
    uint best_seg = std::numeric_limits<uint>::max();
    if (!!target_point) {
      double min_dis = std::numeric_limits<double>::max();
      for (size_t i = 0; i < segments.size(); i++) {
        double dis = geometry::distance(segments.at(i), *target_point);
        if (dis <= min_dis) {
          min_dis = dis;
          best_seg = i;
        }
      }
      ArcCoordinates arc = geometry::toArcCoordinates(segments.at(best_seg), *target_point);
      target_s = arc.length;
    }

    ArcCoordinates arc = geometry::toArcCoordinates(segments.at(0), cur_pos);

    double t_total = 0.0;
    double t = 0.0;
    double s_segment = arc.length;  // s on segment line
    double w;                       // offset
    double d = arc.distance;        // current offset
    double last_d = arc.distance;   // last offset
    uint l = 0;
    double len = geometry::length(segments.at(0));
    bool b_offset = false;

    if (std::fabs(d) > 0.01) {
      b_offset = true;
      w = 0.5 * d;
    }

    while (l < segments.size() && t_total < t_max) {
      bool need_new_segment = len < s_segment;

      if ((l == segments.size() - 1 || l == best_seg) && s_segment > target_s) {
        path_line.push_back(*target_point);
        break;
      }

      if (need_new_segment && l == segments.size() - 1) {  // no more lines, segments or lanelets
        break;
      }

      path_line.push_back(Lanelet2Utilities::fromArcCoordinates_fast(
          segments.at(l), s_segment, d));  // Lanelet2Utilities from new lanelet2_utils.hpp

      if (b_offset) {
        // https://www.desmos.com/calculator/erjar0accp
        // w = 0.5 * w
        d = w * sin(t / t_ref * M_PI + M_PI_2) + w;
      } else {
        d = 0.0;
      }

      if (need_new_segment) {  // lane change
        l++;
        const BasicPoint2d &p = path_line.back();
        const BasicLineString2d &line = segments.at(l);
        w = 0.5 * geometry::signedDistance(line, p) - d / 2;
        len = geometry::length(segments.at(l));
        s_segment = 0.0;
        t = 0.0;
        b_offset = true;
        d = w * sin(t / t_ref * M_PI + M_PI_2) + w;

        if (l == segments.size() - 2) {  // segment before last segment
          d = 0;
          b_offset = false;
        }
      }

      if (b_offset && (std::fabs(d) < 0.005 || t > t_ref)) {
        d = 0.0;
        b_offset = false;
      }

      t += dt;
      t_total += dt;

      double dd = d - last_d;
      double ds = std::sqrt(std::max(0.0, vel * dt * vel * dt - dd * dd));
      s_segment += ds;
      last_d = d;
    }
    return path_line;
  }

  // Creates a linestring from a lanelet path, sampled by distane with step size ds, accounting for lane changes with a sine function
  static BasicLineString2d llPath2llLineDistanceBased(
      const ConstLanelets &ll_path, const BasicPoint2d &cur_pos, const double &vel, const double &t_ref,
      const double &s_max, const double &ds, boost::optional<const BasicPoint2d &> target_point,
      boost::optional<std::pair<BasicLineString2d, BasicLineString2d> &> boundaries = {},
      boost::optional<const routing::RoutingGraph &> routing_graph = {}) {
    BasicLineString2d path_line;
    path_line.push_back(cur_pos);

    // Extract centerlines from lanelets, push or extend entries in segment vector
    std::vector<BasicLineString2d> segments(1), segments_boundary_left(1), segments_boundary_right(1);
    bool wasBicycleBoundary = false;
    for (size_t l = 0; l < ll_path.size(); l++) {
      // Adjacent lanelets (lane change) -> push new segment
      if (l < ll_path.size() - 1 &&
          (geometry::rightOf(ll_path.at(l), ll_path.at(l + 1)) || geometry::leftOf(ll_path.at(l), ll_path.at(l + 1)))) {
        if (l == 0) {
          segments.clear();
          segments_boundary_left.clear();
          segments_boundary_right.clear();
        }
        segments.push_back(ll_path.at(l + 1).centerline2d().basicLineString());
        if (!!boundaries) {
          segments_boundary_left.push_back(ll_path.at(l + 1).leftBound2d().basicLineString());
          segments_boundary_right.push_back(ll_path.at(l + 1).rightBound2d().basicLineString());
        }
        l++;
        continue;
      }

      // Append to path linestring
      const BasicLineString2d line = ll_path.at(l).centerline2d().basicLineString();
      segments.back().insert(segments.back().end(), line.begin() + int(l != 0), line.end());

      // Append to boundary linestrings
      if (!!boundaries) {
        BasicLineString2d bicycle_lane_right;
        if (!!routing_graph) {
          ConstLanelets right_lanelets = routing_graph->rights(ll_path.at(l));
          size_t ll_count = 0;
          for (const auto &ll : right_lanelets) {
            if (ll.hasAttribute(AttributeName::Subtype) &&
                ll.attribute(AttributeName::Subtype) == AttributeValueString::BicycleLane &&
                ((ll.leftBound().hasAttribute(AttributeName::Subtype) &&
                  ll.leftBound().attribute(AttributeName::Subtype) == AttributeValueString::Dashed) ||
                 (ll.leftBound().hasAttribute(AttributeName::Type) &&
                  ll.leftBound().attribute(AttributeName::Type) == AttributeValueString::Virtual))) {
              const BasicLineString2d line_bicycle_lane_right = ll.rightBound2d().basicLineString();
              bicycle_lane_right.insert(bicycle_lane_right.end(), line_bicycle_lane_right.begin() + int(ll_count != 0),
                                        line_bicycle_lane_right.end());
            }
          }
        }
        const BasicLineString2d line_b_left = ll_path.at(l).leftBound2d().basicLineString();
        segments_boundary_left.back().insert(segments_boundary_left.back().end(), line_b_left.begin() + int(l != 0),
                                             line_b_left.end());

        bool offset_right = l != 0;
        if (!bicycle_lane_right.size()) {
          bicycle_lane_right = ll_path.at(l).rightBound2d().basicLineString();
          if (wasBicycleBoundary) {
            offset_right = 0;
          }
          wasBicycleBoundary = false;
        } else {
          if (!wasBicycleBoundary) {
            offset_right = 0;
          }
          wasBicycleBoundary = true;
        }
        segments_boundary_right.back().insert(segments_boundary_right.back().end(),
                                              bicycle_lane_right.begin() + offset_right, bicycle_lane_right.end());
      }
    }

    // Find closest segment to target point (and its arc coordinate)
    double target_s = std::numeric_limits<double>::max();
    uint best_seg = std::numeric_limits<uint>::max();
    if (!!target_point) {
      double min_dis = std::numeric_limits<double>::max();
      for (size_t i = 0; i < segments.size(); i++) {
        double dis = geometry::distance(segments.at(i), *target_point);
        if (dis <= min_dis) {
          min_dis = dis;
          best_seg = i;
        }
      }
      const ArcCoordinates arc = geometry::toArcCoordinates(segments.at(best_seg), *target_point);
      target_s = arc.length;
    }

    // Prepare main loop (handle lane changes)
    ArcCoordinates arc = geometry::toArcCoordinates(segments.at(0), cur_pos);
    double s_total = 0.0;           // s on the global route
    double s_maneuver = 0.0;        // s during lane change maneuver
    double s_segment = arc.length;  // s on current segment line
    double w;                       // offset during lane change maneuver
    double d = arc.distance;        // current offset
    uint l = 0;
    double len = geometry::length(segments.at(0));  // length of segment
    bool b_offset = false;
    double s_ref = t_ref * vel;  // target distance length for lange change maneuver
    double boundary_dist_left = 0.0;
    double boundary_dist_right = 0.0;
    BasicLineString2d tmp_boundary_left, tmp_boundary_right;
    if (std::fabs(d) > 0.01) {
      b_offset = true;
      w = 0.5 * d;
    }

    // Main loop; sampled by distance with ds as step size and handles lane changes
    while (s_total < s_max && l < segments.size()) {
      // Have we reached the target point?
      if ((l == segments.size() - 1 || l == best_seg) && s_segment > target_s) {
        path_line.push_back(*target_point);
        break;
      }

      // No more lines, segments or lanelets?
      const bool need_new_segment = s_segment > len;
      if (need_new_segment && l == segments.size() - 1) {
        break;
      }

      // In case the remaining route length is shorter than the required s for the lane change maneuver
      if (l == best_seg && (target_s - s_segment) < s_ref) {
        s_ref = target_s - s_segment;
      }

      // Approximate lange change maneuver with sine function
      if (b_offset) {
        // https://www.desmos.com/calculator/erjar0accp
        // w = 0.5 * w
        d = w * sin(s_maneuver / s_ref * M_PI + M_PI_2) + w;
      } else {
        d = 0.0;
        s_maneuver = 0.0;
      }

      // Lane change
      if (need_new_segment) {
        l++;
        const BasicPoint2d &p = path_line.back();
        const BasicLineString2d &line = segments.at(l);
        w = 0.5 * geometry::signedDistance(line, p) - d / 2;
        len = geometry::length(segments.at(l));
        s_maneuver = 0.0;
        b_offset = true;
        d = w * sin(s_maneuver / s_ref * M_PI + M_PI_2) + w;

        const BasicPoint2d p_tmp = Lanelet2Utilities::fromArcCoordinates_fast(
            segments.at(l), 0.0, d);  // Lanelet2Utilities from new lanelet2_utils.hpp
        s_segment = ds - geometry::distance(path_line.back(), p_tmp);

        // Last boundary point before lane change
        if (!!boundaries) {
          Lanelet2Utilities::addBoundarySegment(
              *boundaries, std::make_pair(segments_boundary_left[l - 1], segments_boundary_right[l - 1]),
              path_line);  // Lanelet2Utilities from new lanelet2_utils.hpp
          boundary_dist_left = geometry::distance(path_line.back(), boundaries->first.back());
          boundary_dist_right = geometry::distance(path_line.back(), boundaries->second.back());
        }
      }

      // Lane change / offset complete
      bool b_lane_change_complete = false;
      if (b_offset && (std::fabs(d) < 0.005 || s_maneuver > s_ref)) {
        d = 0.0;
        s_maneuver = 0.0;
        b_offset = false;
        b_lane_change_complete = true;
      }

      path_line.push_back(Lanelet2Utilities::fromArcCoordinates_fast(
          segments.at(l), s_segment, d));  // Lanelet2Utilities from new lanelet2_utils.hpp

      if (!!boundaries) {
        // Lane change completed -> remove original boundaries during lane change and insert our manually created boundaries instead
        if (b_lane_change_complete) {
          Lanelet2Utilities::delFromBoundarySegment(segments_boundary_left[l], segments_boundary_right[l],
                                                    path_line);  // Lanelet2Utilities from new lanelet2_utils.hpp
          boundaries->first.insert(boundaries->first.end(), tmp_boundary_left.begin(), tmp_boundary_left.end());
          boundaries->second.insert(boundaries->second.end(), tmp_boundary_right.begin(), tmp_boundary_right.end());
          tmp_boundary_left.clear();
          tmp_boundary_right.clear();
        }
        // Manually create boundaries during lane change
        else if (b_offset) {
          tmp_boundary_left.push_back(
              geometry::internal::lateralShiftPointAtIndex(path_line, path_line.size() - 1, boundary_dist_left));
          tmp_boundary_right.push_back(
              geometry::internal::lateralShiftPointAtIndex(path_line, path_line.size() - 1, -boundary_dist_right));
        }
      }

      s_segment += ds;
      s_maneuver += ds;
      s_total += ds;
    }
    if (!!boundaries && l < segments_boundary_left.size()) {
      Lanelet2Utilities::addBoundarySegment(*boundaries,
                                            std::make_pair(segments_boundary_left[l], segments_boundary_right[l]),
                                            path_line);  // Lanelet2Utilities from new lanelet2_utils.hpp
    }

    return path_line;
  }

  static double computeCurvature(const BasicPoint2d &p1, const BasicPoint2d &p2, const BasicPoint2d &p3) {
    // https://en.wikipedia.org/wiki/Menger_curvature#Definition
    const double area = 0.5 * ((p2.x() - p1.x()) * (p3.y() - p1.y()) - (p2.y() - p1.y()) * (p3.x() - p1.x()));
    const double side1 = geometry::distance(p1, p2);
    const double side2 = geometry::distance(p1, p3);
    const double side3 = geometry::distance(p2, p3);
    const double product = side1 * side2 * side3;
    if (product < 1e-20) {
      return std::numeric_limits<double>::max();
    }
    return 4 * area / product;
  }
};
