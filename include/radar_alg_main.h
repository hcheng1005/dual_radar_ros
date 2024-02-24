#pragma once

#include <iostream>
#include <memory>
#include <string>

#include "common.h"
#include "../src/common_lib/dbscan.h"
#include "../src/common_lib/lshape.h"

namespace RadarExp
{
    class radarAlg
    {
    private:
        bool VISUALIZATION = false;
        /* data */
        std::shared_ptr<std::vector<radarPoint>> radarPointsPtr = nullptr;
        std::vector<Bndbox> Bndboxs;

        void getRadarClusters();

        void getDetBox(const std::vector<DBSCAN::Point4DBSCAN> PointSet,
                       const std::vector<std::vector<size_t>> &clusterSet);

    public:
        radarAlg(){};
        ~radarAlg(){};

        void proc(std::vector<radarPoint> &radarPoints);

        std::vector<Bndbox>* getBoxesOutput(){return &Bndboxs;};
    };
}