#ifndef HDL_GLOBAL_LOCALIZATION_NEW_BBS_LOCALIZATION_HPP
#define HDL_GLOBAL_LOCALIZATION_NEW_BBS_LOCALIZATION_HPP

#include <hdl_global_localization/bbs/bbs_localization.hpp>

namespace hdl_global_localization
{

class NewBBSLocalization {
public:
  using Points = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

  NewBBSLocalization(const BBSParams& params = BBSParams());

  void set_map(const Points& map_points, double resolution, int width, int height, int pyramid_levels, int max_points_per_cell);

  boost::optional<Eigen::Isometry2f> localize(const Points& scan_points, double min_score, double* best_score = nullptr);

  std::shared_ptr<const OccupancyGridMap> gridmap() const;

private:
  std::priority_queue<DiscreteTransformation> create_init_transset(const Points& scan_points) const;

private:
  BBSParams params;

  double theta_resolution;
  std::vector<std::shared_ptr<OccupancyGridMap>> gridmap_pyramid;
};

} // namespace hdl_global_localization


#endif // HDL_GLOBAL_LOCALIZATION_NEW_BBS_LOCALIZATION_HPP