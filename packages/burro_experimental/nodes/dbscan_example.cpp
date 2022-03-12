#include <iostream>
#include <cstdio>
#include <random>

#include <burro_exp/ml/dbscan.h>

#include <burro/time/timing.h>
#include <burro/util/logging.h>



struct vec3f {
    float data[3];
    float operator[](int idx) const { return data[idx]; }
};

int main(int argc, char* argv[]) {

    int N = 5000;
    int num_clusters = 4;

    std::mt19937 rng;
    rng.seed(0);

    std::normal_distribution<float> center_rng(0, 2);
    std::normal_distribution<float> cluster_rng(0, 0.2);

    std::vector<Point> dbdataset;
    std::vector<vec3f> dbdataset2;

    // Generate some data
    for (int i = 0; i < num_clusters; i++)
    {
        float x_i = center_rng(rng);
        float y_i = center_rng(rng);
        float z_i = center_rng(rng);
        SPDLOG_INFO("Cluster {}: {:2f}, {:2f}, {:2f}", i, x_i, y_i, z_i);

        for (int j = i * N; j < (i + 1) * N; j++)
        {
            Point pt{x_i + cluster_rng(rng), y_i + cluster_rng(rng), z_i + cluster_rng(rng), UNCLASSIFIED};
            dbdataset.push_back(pt);

            dbdataset2.push_back(vec3f{pt.data[0], pt.data[1], pt.data[2]});
        }
    }

    SPDLOG_INFO("Dataset Size: {}", dbdataset.size());

    burro::time::Timer timer2;
    jamesyoo::DBSCAN ds(10, 0.1, dbdataset);

    ds.run();
    SPDLOG_INFO("DBSCAN (james-yoo) time: {:.3f}s", burro::time::ToSeconds(timer2.Elapsed()));


    burro::time::Timer timer3;
    auto dbscan = jiang::DBSCAN<vec3f, float>();

    dbscan.Run(&dbdataset2, 3, 0.5f, 10);
    auto noise = dbscan.Noise;
    auto clusters = dbscan.Clusters;
    SPDLOG_INFO("DBSCAN (jiang) time: {:.3f}s", burro::time::ToSeconds(timer3.Elapsed()));

    printResults(ds.m_points, ds.getTotalPointSize());

    return 0;
}
