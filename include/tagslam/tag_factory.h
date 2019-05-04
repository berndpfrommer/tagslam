/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/tag2.h"
namespace tagslam {
  class TagFactory {
  public:
    virtual ~TagFactory() {};
    virtual Tag2ConstPtr findTag(int tagId) = 0;
  };
}
