// Copyright (c) 2011 libmv authors.
#include "klt.h"
#include "libmv/correspondence/matches.h"
#include "libmv/correspondence/feature.h"
#include "libmv/correspondence/klt.h"
#include "libmv/image/image.h"
#include "libmv/image/image_io.h"
#include "libmv/image/image_pyramid.h"
#include "libmv/image/image_sequence_io.h"
#include "libmv/image/cached_image_sequence.h"
#include "libmv/image/pyramid_sequence.h"
using namespace libmv;

class KLTPrivate : public KLT {
private:
    libmv::ImageSequence* source_;
    libmv::PyramidSequence* pyramid_sequence_;
    libmv::KLTContext klt_;
    std::list<libmv::KLTPointFeature*> features_;
    libmv::Matches matches_;
public:
    KLTPrivate(std::vector<std::string> files, int pyramid_levels, float sigma) {
        source_ = ImageSequenceFromFiles(files,new ImageCache());
        pyramid_sequence_ = MakePyramidSequence(source_, pyramid_levels, sigma);

        klt_.DetectGoodFeatures(pyramid_sequence_->Pyramid(0)->Level(0), &features_);
        int i = 0;
        for (KLTContext::FeatureList::iterator it = features_.begin(); it != features_.end(); ++it, ++i) {
            matches_.Insert(0, i, *it);
        }
    }
    void Process(int i) {
        for (Matches::Features<KLTPointFeature> r = matches_.InImage<KLTPointFeature>(i-1); r; ++r) {
            KLTPointFeature *next_position = new KLTPointFeature;
            if (klt_.TrackFeature(pyramid_sequence_->Pyramid(i-1), *r.feature(),
                                  pyramid_sequence_->Pyramid(i), next_position)) {
                matches_.Insert(i, r.track(), next_position);
            } else {
                delete next_position; //FIXME
            }
        }
    }
};
KLT* KLT::Load(std::vector<std::string> files, int pyramid_levels, float sigma) { return new KLTPrivate(files,pyramid_levels,sigma); }
