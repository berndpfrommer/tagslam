/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/tag.h"
namespace tagslam {
  class TagFactory {
  public:
    virtual ~TagFactory() {};
    virtual TagConstPtr findTag(int tagId) = 0;
  };
}
