#ifndef DBSCAN_H
#define DBSCAN_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#define DIMENSIONS 16
#define P_NORM 2

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2

#define SUCCESS 0
#define FAILURE -3

using namespace std;

typedef struct Point_
{
    float x[DIMENSIONS];
    int clusterID;
} Point;

typedef struct _Cluster
{
    unsigned int size;
    unsigned int id;
} Cluster;

vector<float> main_centroid(vector<Point> &points);

class DBSCAN
{
public:
    DBSCAN(unsigned int minPts, float eps, vector<Point> &points)
    {
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }
    ~DBSCAN() {}

    int run();
    vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    inline double calculateDistance(Point pointCore, Point pointTarget);

    int getTotalPointSize() { return m_pointSize; }
    int getMinimumClusterSize() { return m_minPoints; }
    int getEpsilonSize() { return m_epsilon; }

    vector<Point> m_points;
private:
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
};

#endif
