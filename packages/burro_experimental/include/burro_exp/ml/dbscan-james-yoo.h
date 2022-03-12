#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include <set>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;

template<size_t DIM=3>
struct PointBase
{
    float data[DIM];  // X, Y, Z position
    int clusterID;  // clustered ID

    bool operator==(const PointBase<DIM>& other) const
    {
        for(size_t d = 0; d < DIM; d++)
        {
            if (data[d] != other.data[d]) return false;
        }
        return true;
    }
};

using Point = PointBase<3>;

template<size_t DIM>
inline double calculateDistance(const PointBase<DIM>& p1, const PointBase<DIM>& p2)
{
    double sum(0);
    for (size_t d = 0; d < DIM; d++)
    {
        sum += pow(p1.data[d] - p2.data[d], 2);
    }
    return sum;
}


template<>
inline double calculateDistance(const PointBase<2>& p1, const PointBase<2>& p2)
{
    return pow(p1.data[0] - p2.data[0], 2) + pow(p1.data[1] - p2.data[1], 2);
}


template<>
inline double calculateDistance(const PointBase<3>& p1, const PointBase<3>& p2)
{
    return pow(p1.data[0] - p2.data[0], 2) + pow(p1.data[1] - p2.data[1], 2) + pow(p1.data[2] - p2.data[2], 2);
}


namespace jamesyoo
{


class DBSCAN {
public:
    DBSCAN(unsigned int minPts, float eps, vector<Point> points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }
    ~DBSCAN(){}

    int run()
    {
        int clusterID = 1;
        for(auto& p: m_points)
        {
            if (p.clusterID == UNCLASSIFIED && expandCluster(p, clusterID) != FAILURE)
            {
                clusterID++;
            }
        }

        return 0;
    }

    vector<int> calculateCluster(const Point& point)
    {
        int index = 0;
        vector<int> clusterIndex;
        for(auto& p : m_points)
        {
            if ( calculateDistance(point, p) <= m_epsilon )
            {
                clusterIndex.push_back(index);
            }
            index++;
        }
        return clusterIndex;
    }

    int expandCluster(Point& point, int clusterID)
    {
        vector<int> clusterSeeds = calculateCluster(point);

        if ( clusterSeeds.size() < m_minPoints )
        {
            point.clusterID = NOISE;
            return FAILURE;
        }
        else
        {
            int index = 0, indexCorePoint = 0;
            for(auto& seed : clusterSeeds)
            {
                m_points.at(seed).clusterID = clusterID;
                if (m_points.at(seed) == point)
                {
                    indexCorePoint = index;
                }
                ++index;
            }
            clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);

            for( vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
            {
                vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

                if ( clusterNeighors.size() >= m_minPoints )
                {
                    for ( auto& neighbor : clusterNeighors)
                    {
                        if ( m_points.at(neighbor).clusterID == UNCLASSIFIED || m_points.at(neighbor).clusterID == NOISE )
                        {
                            if ( m_points.at(neighbor).clusterID == UNCLASSIFIED )
                            {
                                clusterSeeds.push_back(neighbor);
                                n = clusterSeeds.size();
                            }
                            m_points.at(neighbor).clusterID = clusterID;
                        }
                    }
                }
            }

            return SUCCESS;
        }
    }

    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}

public:
    vector<Point> m_points;

private:
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
};

}

void printResults(vector<Point>& points, int num_points)
{
    std::set<int> labels;
    for (auto& p : points)
    {
          labels.insert(p.clusterID);
    }

    for (auto& l : labels)
    {
        std::cout << l << ", ";
    }

    std::cout << std::endl;

}

#endif // DBSCAN_H
