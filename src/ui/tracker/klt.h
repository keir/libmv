#pragma once
#include <list>
#include <vector>
#include <string>

namespace libmv {
class ImageSequence;
class PyramidSequence;
class KLTContext;
class KLTPointFeature;
class Matches;
}

class KLT {
public:
    KLT(std::vector<std::string> files, int pyramid_levels=4, float sigma=0.9);
    void process(int frame);
private:
    libmv::ImageSequence *source;
    libmv::PyramidSequence *pyramid_sequence;
    libmv::KLTContext *klt;
    std::list<libmv::KLTPointFeature*> *features;
    libmv::Matches *matches;
};
