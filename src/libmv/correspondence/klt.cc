// Copyright (c) 2007, 2008 libmv authors.
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

#include <cassert>

#include "libmv/base/vector.h"
#include "libmv/numeric/numeric.h"
#include "libmv/correspondence/klt.h"
#include "libmv/image/image.h"
#include "libmv/image/image_io.h"
#include "libmv/image/convolve.h"
#include "libmv/image/sample.h"

using std::max;
using std::min;

namespace libmv {

static void FindLocalMaxima(const FloatImage &trackness,
                            float min_trackness,
                            KLTContext::FeatureList *features) {
  for (int i = 1; i < trackness.Height()-1; ++i) {
    for (int j = 1; j < trackness.Width()-1; ++j) {
      if (   trackness(i,j) >= min_trackness
          && trackness(i,j) >= trackness(i-1, j-1)
          && trackness(i,j) >= trackness(i-1, j  )
          && trackness(i,j) >= trackness(i-1, j+1)
          && trackness(i,j) >= trackness(i  , j-1)
          && trackness(i,j) >= trackness(i  , j+1)
          && trackness(i,j) >= trackness(i+1, j-1)
          && trackness(i,j) >= trackness(i+1, j  )
          && trackness(i,j) >= trackness(i+1, j+1)) {
        KLTPointFeature p;
        p.coords(1) = i;
        p.coords(0) = j;
        p.trackness = trackness(i,j);
        features->push_back(p);
      }
    }
  }
}

// Compute the gradient matrix noted by Z in Good Features to Track.
//
//   Z = [gxx gxy; gxy gyy]
//
// This function computes the matrix for every pixel.
static void ComputeGradientMatrix(const Array3Df &image_and_gradients,
                                       int window_size,
                                       Array3Df *gradient_matrix) {
  Array3Df gradients;
  gradients.ResizeLike(image_and_gradients);
  for (int j = 0; j < image_and_gradients.Height(); ++j) {
    for (int i = 0; i < image_and_gradients.Width(); ++i) {
      float gx = image_and_gradients(j, i, 1);
      float gy = image_and_gradients(j, i, 2);
      gradients(j, i, 0) = gx * gx;
      gradients(j, i, 1) = gx * gy;
      gradients(j, i, 2) = gy * gy;
    }
  }
  // Sum the gradient matrix over tracking window for each pixel.
  BoxFilter(gradients, window_size, gradient_matrix);
}

// Given the three distinct elements of the symmetric 2x2 matrix
//
//                     [gxx gxy]
//                     [gxy gyy],
//
// return the minimum eigenvalue of the matrix.
// Borrowed from Stan Birchfield's KLT implementation.
static float MinEigenValue(float gxx, float gxy, float gyy) {
  return (gxx + gyy - sqrt((gxx - gyy) * (gxx - gyy) + 4 * gxy * gxy)) / 2.0f;
}

// Compute trackness of every pixel given the gradient matrix.
// This is done as described in the Good Features to Track paper.
static void ComputeTrackness(const Array3Df gradient_matrix,
                             Array3Df *trackness_pointer,
                             double *trackness_mean) {
  Array3Df &trackness = *trackness_pointer;
  trackness.Resize(gradient_matrix.Height(), gradient_matrix.Width());
  *trackness_mean = 0;
  for (int i = 0; i < trackness.Height(); ++i) {
    for (int j = 0; j < trackness.Width(); ++j) {
      double t = MinEigenValue(gradient_matrix(i, j, 0),
                               gradient_matrix(i, j, 1),
                               gradient_matrix(i, j, 2));
      trackness(i, j) = t;
      *trackness_mean += t;
    }
  }
  *trackness_mean /= trackness.Size();
}

// TODO(keir): Use Stan's neat trick of using a 'punch-out' array to detect
// too-closes features. This is O(n^2)!
static void RemoveTooCloseFeatures(KLTContext::FeatureList *features,
                                   double squared_min_distance) {
  // mark close features with least trackness as invalid
  for(size_t i = 0 ; i < features->size() ; ++i) {
    KLTPointFeature& a = features->at(i);
    for(size_t j = 0 ; j < i ; j++) {  // compare each feature pair once
      KLTPointFeature& b = features->at(j);
      if ( b.trackness != 0  // skip invalidated features
           && (a.coords-b.coords).squaredNorm() < squared_min_distance ) {
        ((a.trackness < b.trackness) ? a : b).trackness=0;  // invalid feature
      }
    }
  }
  // compress feature array in place by removing invalid features
  size_t size = 0;
  for(size_t i = 0 ; i < features->size() ; ++i) {
    const KLTPointFeature& a = features->at(i);
    if( a.trackness != 0 ) features->at(size++) = a;
  }
  features->resize(size);
}

void KLTContext::DetectGoodFeatures(const Array3Df &image_and_gradients,
                                    FeatureList *features) {
  Array3Df gradient_matrix;
  ComputeGradientMatrix(image_and_gradients, WindowSize(), &gradient_matrix);

  Array3Df trackness;
  double trackness_mean;
  ComputeTrackness(gradient_matrix, &trackness, &trackness_mean);
  min_trackness_ = trackness_mean;

  FindLocalMaxima(trackness, min_trackness_, features);

  RemoveTooCloseFeatures(features, min_feature_distance_ * min_feature_distance_);
}

void KLTContext::TrackFeatures(ImagePyramid *pyramid1,
                               const FeatureList &features1,
                               ImagePyramid *pyramid2,
                               FeatureList *features2) {
  features2->clear();
  for (size_t i = 0; i < features1.size(); ++i) {
    KLTPointFeature tracked_feature;
    TrackFeature(pyramid1, features1.at(i), pyramid2, &tracked_feature);
    features2->push_back(tracked_feature);
  }
}

bool KLTContext::TrackFeature(ImagePyramid *pyramid1,
                              const KLTPointFeature &feature1,
                              ImagePyramid *pyramid2,
                              KLTPointFeature *feature2) {
  Vec2 position1, position2;
  position2(0) = feature1.coords(0);
  position2(1) = feature1.coords(1);
  position2 /= pow(2., pyramid1->NumLevels());

  for (int i = pyramid1->NumLevels() - 1; i >= 0; --i) {
    position1(0) = feature1.coords(0) / pow(2., i);
    position1(1) = feature1.coords(1) / pow(2., i);
    position2 *= 2;

    bool succeeded = TrackFeatureOneLevel(pyramid1->Level(i),
                                          position1,
                                          pyramid2->Level(i),
                                          &position2);
    if (i == 0 && !succeeded) {
      // Only fail on the highest-resolution level, because a failure on a
      // coarse level does not mean failure at a lower level (consider
      // out-of-bounds conditions).
      return false;
    }
  }
  feature2->coords = position2.cast<float>();
  return true;
}

// Compute the gradient matrix noted by Z and the error vector e.
// See Good Features to Track.
static void ComputeTrackingEquation(const Array3Df &image_and_gradient1,
                                    const Array3Df &image_and_gradient2,
                                    const Vec2 &position1,
                                    const Vec2 &position2,
                                    int half_width,
                                    float *gxx,
                                    float *gxy,
                                    float *gyy,
                                    float *ex,
                                    float *ey) {
  *gxx = *gxy = *gyy = 0;
  *ex = *ey = 0;
  for (int r = -half_width; r <= half_width; ++r) {
    for (int c = -half_width; c <= half_width; ++c) {
      float x1 = position1(0) + c;
      float y1 = position1(1) + r;
      float x2 = position2(0) + c;
      float y2 = position2(1) + r;
      // TODO(pau): should do boundary checking outside this loop, and call here
      // a sampler that does not boundary checking.
      float I =  SampleLinear(image_and_gradient1, y1, x1, 0);
      float J =  SampleLinear(image_and_gradient2, y2, x2, 0);
      float gx = SampleLinear(image_and_gradient2, y2, x2, 1);
      float gy = SampleLinear(image_and_gradient2, y2, x2, 2);
      *gxx += gx * gx;
      *gxy += gx * gy;
      *gyy += gy * gy;
      *ex += (I - J) * gx;
      *ey += (I - J) * gy;
    }
  }
}

// Solve the tracking equation
//
//   [gxx gxy] [dx] = [ex]
//   [gxy gyy] [dy] = [ey]
//
// for dx and dy.  Borrowed from Stan Birchfield's KLT implementation.
//
// TODO(keir): Replace this with calls to eigen (faster, more stable).
static bool SolveTrackingEquation(float gxx, float gxy, float gyy,
                                  float ex, float ey,
                                  float min_determinant,
                                  float *dx, float *dy) {
  float det = gxx * gyy - gxy * gxy;
  if (det < min_determinant) {
    *dx = 0;
    *dy = 0;
    return false;
  }
  *dx = (gyy * ex - gxy * ey) / det;
  *dy = (gxx * ey - gxy * ex) / det;
  return true;
}

bool KLTContext::TrackFeatureOneLevel(const Array3Df &image_and_gradient1,
                                      const Vec2 &position1,
                                      const Array3Df &image_and_gradient2,
                                      Vec2 *position2) {
  int i;
  float dx=0, dy=0;
  for (i = 0; i < max_iterations_; ++i) {
    // Compute gradient matrix and error vector.
    float gxx, gxy, gyy, ex, ey;
    ComputeTrackingEquation(image_and_gradient1, image_and_gradient2,
                            position1, *position2,
                            HalfWindowSize(),
                            &gxx, &gxy, &gyy, &ex, &ey);
    // Solve the linear system for deltad.
    if (!SolveTrackingEquation(gxx, gxy, gyy, ex, ey, min_determinant_,
                               &dx, &dy)) {
      return false;
    }

    // Update feature2 position.
    (*position2)(0) += dx;
    (*position2)(1) += dy;

    // TODO(keir): Handle other tracking failure conditions and pass the
    // reasons out to the caller. For example, for pyramid tracking a failure
    // at a coarse level suggests trying again at a finer level.
    if (Square(dx) + Square(dy) < min_update_squared_distance_) {
      break;
    }
  }

  if (i == max_iterations_) {
    // TODO(keir): Somehow indicate that we hit max iterations.
  }
  return true;
}

}  // namespace libmv
