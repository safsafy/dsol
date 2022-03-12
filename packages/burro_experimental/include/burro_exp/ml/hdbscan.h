
 // https://github.com/scottnothing/hdbscan-cpp

#include "hdbscan/Runner/hdbscanRunner.hpp"
#include "hdbscan/Runner/hdbscanParameters.hpp"
#include "hdbscan/Runner/hdbscanResult.hpp"

namespace burro
{

class Hdbscan
{
public:
    using Dataset = std::vector<std::vector<double> >;

    Hdbscan(int min_points, int min_cluster_size, const std::string& distance_metric)
    {
        parameters_.minPoints = min_points;
        parameters_.minClusterSize = min_cluster_size;
        parameters_.distanceFunction = distance_metric;
    }

    hdbscanResult Cluster(const Dataset& dataset)
    {
        parameters_.dataset = dataset;
        return runner_.run(parameters_);
    }


    hdbscanResult Cluster(const hdbscanParameters& parameters)
    {
        parameters_ = parameters;
        return runner_.run(parameters_);
    }

private:
    hdbscanRunner runner_;
    hdbscanParameters parameters_;
};

}