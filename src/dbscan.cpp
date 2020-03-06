#include "dbscan.h"

int DBSCAN::run()
{
    int clusterID = 1;
    vector<Point>::iterator iter;
    for (iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if (iter->clusterID == UNCLASSIFIED)
        {
            if (expandCluster(*iter, clusterID) != FAILURE)
            {
                clusterID += 1;
            }
        }
    }

    return 0;
}

int DBSCAN::expandCluster(Point point, int clusterID)
{
    vector<int> clusterSeeds = calculateCluster(point);

    if (clusterSeeds.size() < m_minPoints)
    {
        point.clusterID = NOISE;
        return FAILURE;
    }
    else
    {
        int index = 0, indexCorePoint = 0;
        vector<int>::iterator iterSeeds;
        for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
            m_points.at(*iterSeeds).clusterID = clusterID;
            bool equal = true;
            for(int i =0; i<DIMENSIONS && equal; i++) 
                equal &= (m_points.at(*iterSeeds).x[i] == point.x[i]);
            if (equal)
            {
                indexCorePoint = index;
            }
            ++index;
        }
        clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

        for (vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
        {
            vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

            if (clusterNeighors.size() >= m_minPoints)
            {
                vector<int>::iterator iterNeighors;
                for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
                {
                    if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE)
                    {
                        if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED)
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points.at(*iterNeighors).clusterID = clusterID;
                    }
                }
            }
        }
        return SUCCESS;
    }
}

vector<int> DBSCAN::calculateCluster(Point point)
{
    int index = 0;
    vector<Point>::iterator iter;
    vector<int> clusterIndex;
    for (iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if (calculateDistance(point, *iter) <= m_epsilon)
        {
            clusterIndex.push_back(index);
        }
        index++;
    }
    return clusterIndex;
}

inline double DBSCAN::calculateDistance(Point pointCore, Point pointTarget)
{
    float sum = 0;
    for (int i = 0; i < DIMENSIONS; i++)
        sum += pow(pointCore.x[i] - pointTarget.x[i], P_NORM);
    return pow(sum, 1.0/P_NORM);
}

vector<float> main_centroid(vector<Point> &points) {
    const int max_clusters = 100;
    auto clusters_size = (int*)calloc(max_clusters, sizeof(int));
	for (int i=0; i<points.size(); i++) {
        if(points[i].clusterID > 0)
		    clusters_size[points[i].clusterID]++;
    }
	vector<Cluster> clusters=vector<Cluster>();
	for(int i=0; i<max_clusters; i++) {
		if(clusters_size[i] > 0) {
			Cluster c;
			c.id = i;
			c.size = clusters_size[i];
			clusters.push_back(c);
		}
	}
    free(clusters_size);

	sort(clusters.begin(), clusters.end(), 
		[](const Cluster & a, const Cluster & b) -> bool {return a.size > b.size;});
	cout<<endl<<"cluster size:"<<clusters[0].size<<"    id: "<<clusters[0].id<<endl;
    
    int main_cluster_id = clusters[0].id;
    int main_cluster_size = clusters[0].size;
    vector<float> acc = vector<float>(DIMENSIONS, 0);
    for(int i=0; i<points.size(); i++) {
        if(points[i].clusterID != main_cluster_id)
            continue;
        for(int j=0; j<DIMENSIONS; j++) 
            acc[j] += points[i].x[j];
    }
    for(int j=0; j<DIMENSIONS; j++) 
        acc[j] /= main_cluster_size;
    
    return acc;
}
