// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Eigen/Dense>
#include <tagslam/geometry.hpp>

namespace tagslam
{
namespace quartic
{
typedef std::complex<double> Complex;
int solve_linear(const double a, const double b, Complex * root);
int solve_quadratic(
  const double a, const double b, const double c, Complex * root);
int solve_cubic(
  const double a, const double b, const double c, const double d,
  Complex * root);

int solve_quartic(
  const double a, const double b, const double c, const double d,
  const double e, Complex * root);
}  // namespace quartic
}  // namespace tagslam
