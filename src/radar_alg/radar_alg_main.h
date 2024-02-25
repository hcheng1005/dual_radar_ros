#pragma once

#include <iostream>
#include <memory>
#include <string>


#include "common.h"
#include "../common_lib/dbscan.h"
#include "../common_lib/lshape.h"

#include "./ego_velocity_estimator.h"

namespace RadarExp
{
    class radarAlg
    {
    private:
        bool VISUALIZATION = false;
        /* data */
        std::shared_ptr<std::vector<radarPoint>> radarPointsPtr = nullptr;
        std::vector<Bndbox> Bndboxs;

        void pointsFilter(std::shared_ptr<std::vector<radarPoint>> &pointPtr);

        void EgoVelocityEstimator(std::vector<radarPoint> &radarPoints);

        void getRadarClusters();

        void getDetBox(const std::vector<DBSCAN::Point4DBSCAN> PointSet,
                       const std::vector<std::vector<size_t>> &clusterSet);

    public:
        radarAlg(){
            RadarEgoVelocityEstimatorPtr = new rio::RadarEgoVelocityEstimator();
        };
        ~radarAlg(){};

        void proc(std::vector<radarPoint> &radarPoints);

        rio::RadarEgoVelocityEstimator *RadarEgoVelocityEstimatorPtr = nullptr;

        std::vector<Bndbox>* getBoxesOutput(){return &Bndboxs;};
    };
}