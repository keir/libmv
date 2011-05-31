#pragma once
#include <vector>
#include <string>

class KLT {
public:
    static KLT* Load(std::vector<std::string> files, int pyramid_levels=4, float sigma=0.9);
    virtual void Process(int frame) = 0;
    //TODO: virtual std::vector<Feature> GetFeatures(int frame) = 0;
};
