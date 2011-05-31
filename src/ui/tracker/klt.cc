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

KLT::KLT(std::vector<std::string> files, int pyramid_levels, float sigma) {
    source = ImageSequenceFromFiles(files,new ImageCache());
    pyramid_sequence = MakePyramidSequence(source, pyramid_levels, sigma);
    klt = new KLTContext();
    features = new KLTContext::FeatureList();
    matches = new Matches();

    klt->DetectGoodFeatures(pyramid_sequence->Pyramid(0)->Level(0), features);
    int i = 0;
    for (KLTContext::FeatureList::iterator it = features->begin(); it != features->end(); ++it, ++i) {
        matches->Insert(0, i, *it);
    }
}

void KLT::process(int i) {
    //printf("Tracking %2zd features in %s\n", features.size(), files[i].c_str());
    for (Matches::Features<KLTPointFeature> r = matches->InImage<KLTPointFeature>(i-1); r; ++r) {
        KLTPointFeature *next_position = new KLTPointFeature;
        if (klt->TrackFeature(pyramid_sequence->Pyramid(i-1), *r.feature(),
                             pyramid_sequence->Pyramid(i), next_position)) {
            matches->Insert(i, r.track(), next_position);
        } else {
            delete next_position; //FIXME
        }
    }
    /*for (; features; ++features) {
      DrawFeature(*features.feature(), green, &output_image);
    }*/
}
