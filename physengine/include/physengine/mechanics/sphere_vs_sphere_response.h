#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H

#include "../bits/types.h"

// stl
#include <utility>

namespace dte3607::physengine::mechanics
{

  inline std::pair<types::Vector3, types::Vector3>
  computeImpactResponseSphereSphere([[maybe_unused]] types::Point3 const&  s1_p,
                                    [[maybe_unused]] types::Vector3 const& s1_v,
                                    [[maybe_unused]] types::ValueType s1_mass,
                                    [[maybe_unused]] types::Point3 const&  s2_p,
                                    [[maybe_unused]] types::Vector3 const& s2_v,
                                    [[maybe_unused]] types::ValueType s2_mass)
  {
    auto const s1_to_s2 = s2_p - s1_p;
    auto const d
      = s1_to_s2
        / sqrt(std::pow(s1_to_s2[0], 2.0) + std::pow(s1_to_s2[1], 2.0)
               + std::pow(s1_to_s2[2], 2.0));
    auto const inner_v_1_d = blaze::inner(s1_v, d);
    auto const inner_v_2_d = blaze::inner(s2_v, d);
    auto const v_1_n       = s1_v - inner_v_1_d * d;
    auto const v_2_n       = s2_v - inner_v_2_d * d;
    auto const v_1_d_prime
      = ((s1_mass - s2_mass) / (s1_mass + s2_mass)) * inner_v_1_d
        + ((2 * s2_mass) / (s1_mass + s2_mass)) * inner_v_2_d;
    auto const v_2_d_prime
      = ((s2_mass - s1_mass) / (s1_mass + s2_mass)) * inner_v_2_d
        + ((2 * s1_mass) / (s1_mass + s2_mass)) * inner_v_1_d;

    auto const new_v_1 = v_1_n + v_1_d_prime * d;
    auto const new_v_2 = v_2_n + v_2_d_prime * d;

    return std::make_pair(new_v_1, new_v_2);
  }


}   // namespace dte3607::physengine::mechanics


#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
