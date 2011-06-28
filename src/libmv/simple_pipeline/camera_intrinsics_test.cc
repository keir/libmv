// Copyright (c) 2011 libmv authors.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include <iostream>

#include "testing/testing.h"
#include "libmv/simple_pipeline/camera_intrinsics.h"
#include "libmv/logging/logging.h"

namespace libmv {

TEST(CameraIntrinsics, ApplyIsInvertibleSimple) {
  CameraIntrinsics intrinsics;
  intrinsics.SetFocalLength(500.0);
  intrinsics.set_principal_point(250.0, 125.0);
  intrinsics.set_radial_distortion(0.034, 0.0, 0.0);

  // Scan over image coordinates, invert the intrinsics, then re-apply them to
  // make sure the cycle gets back where it started.
  for (double y = 0; y < 1000; y += 100) {
    for (double x = 0; x < 1000; x += 100) {
      double normalized_x, normalized_y;
      intrinsics.InvertIntrinsics(x, y, &normalized_x, &normalized_y);

      double xp, yp;
      intrinsics.ApplyIntrinsics(normalized_x, normalized_y, &xp, &yp);

      EXPECT_NEAR(x, xp, 1e-8) << "y: " << y;
      EXPECT_NEAR(y, yp, 1e-8) << "x: " << x;
      LG << "Error x: " << (x - xp);
      LG << "Error y: " << (y - yp);
    }
  }
}

} // namespace libmv
