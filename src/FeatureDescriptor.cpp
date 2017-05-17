#include "FeatureDescriptor.h"


FeatureDescriptor::FeatureDescriptor()
{
    //ctor
}

FeatureDescriptor::~FeatureDescriptor()
{
    //dtor
}

void FeatureDescriptor::describeEsfFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud, Feature* esfFeature)
{
    // Estimating esf for the cluster
    pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
    pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf_est;
    esf_est.setInputCloud(segmentCloud);
    esf_est.compute(*descriptor);

    // Making esf signature as a vector of floats
    esfFeature->signatureAsVector.resize(640);
    copy(descriptor->points[0].histogram, descriptor->points[0].histogram + 640, esfFeature->signatureAsVector.begin());
    //cerr << esfFeature->signatureAsVector.size() <<endl;
    /*
    vector <pcl::PCLPointField> fields;
    int esf_idx;
    esf_idx = pcl::getFieldIndex (*descriptor, "esf", fields);
    for (size_t i = 0; i < fields[esf_idx].count; ++i){
        esfFeature->signatureAsVector[i] = descriptor->points[0].histogram[i];
    }
    //*/
}

void FeatureDescriptor::describeEigenFeature(Segment* segment, Feature* eigenFeature)
{
    // Find the variances.
    const size_t kNPoints = segment->getSegmentCloud()->points.size();
    pcl::PointCloud<pcl::PointXYZ> variances;
    for (size_t i = 0u; i < kNPoints; ++i)
    {
        variances.push_back(pcl::PointXYZ());
        variances.points[i].x = segment->getSegmentCloud()->points[i].x - segment->getCentroid()[0];
        variances.points[i].y = segment->getSegmentCloud()->points[i].y - segment->getCentroid()[1];
        variances.points[i].z = segment->getSegmentCloud()->points[i].z - segment->getCentroid()[2];
    }

    // Find the covariance matrix. Since it is symmetric, we only bother with the upper diagonal.
    static const int temp_r[] = {0,0,0,1,1,2};
    static const int temp_c[] = {0,1,2,1,2,2};
    const std::vector<size_t> row_indices_to_access(temp_r, temp_r + sizeof(temp_r)/sizeof(temp_r[0]));
    const std::vector<size_t> col_indices_to_access(temp_c, temp_c + sizeof(temp_c)/sizeof(temp_c[0]));
    Eigen::Matrix3f covariance_matrix;
    for (size_t i = 0u; i < row_indices_to_access.size(); ++i)
    {
        const size_t row = row_indices_to_access[i];
        const size_t col = col_indices_to_access[i];
        double covariance = 0;
        for (size_t k = 0u; k < kNPoints; ++k)
        {
            covariance += variances.points[k].data[row] * variances.points[k].data[col];
        }
        covariance /= kNPoints;
        covariance_matrix(row,col) = covariance;
        covariance_matrix(col,row) = covariance;
    }

    // Compute eigenvalues of covariance matrix.
    bool compute_eigenvectors = false;
    Eigen::EigenSolver<Eigen::Matrix3f> eigenvalues_solver(covariance_matrix, compute_eigenvectors);
    std::vector<float> eigenvalues(3, 0.0);
    eigenvalues.at(0) = eigenvalues_solver.eigenvalues()[0].real();
    eigenvalues.at(1) = eigenvalues_solver.eigenvalues()[1].real();
    eigenvalues.at(2) = eigenvalues_solver.eigenvalues()[2].real();
    if (eigenvalues_solver.eigenvalues()[0].imag() != 0.0 ||
      eigenvalues_solver.eigenvalues()[1].imag() != 0.0 ||
      eigenvalues_solver.eigenvalues()[2].imag() != 0.0 )
    {
        cerr << "Eigenvalues should not have non-zero imaginary component." <<endl;
    }

    // Sort eigenvalues from smallest to largest.
    sort(eigenvalues.begin(), eigenvalues.end());

    // Normalize eigenvalues.
    double sum_eigenvalues = eigenvalues.at(0) + eigenvalues.at(1) + eigenvalues.at(2);
    double e1 = eigenvalues.at(0) / sum_eigenvalues;
    double e2 = eigenvalues.at(1) / sum_eigenvalues;
    double e3 = eigenvalues.at(2) / sum_eigenvalues;
    if(e1 == e2 || e2 == e3 || e1 == e3)
        cerr << "Eigenvalues should not be equal." <<endl;

    // Store inside features.
    const double sum_of_eigenvalues = e1 + e2 + e3;
    const double kOneThird = 1.0/3.0;

    const double kNormalizationPercentile = 1.0;

    const double kLinearityMax = 28890.9 * kNormalizationPercentile;
    const double kPlanarityMax = 95919.2 * kNormalizationPercentile;
    const double kScatteringMax = 124811 * kNormalizationPercentile;
    const double kOmnivarianceMax = 0.278636 * kNormalizationPercentile;
    const double kAnisotropyMax = 124810 * kNormalizationPercentile;
    const double kEigenEntropyMax = 0.956129 * kNormalizationPercentile;
    const double kChangeOfCurvatureMax = 0.99702 * kNormalizationPercentile;

    const double kNPointsMax = 13200 * kNormalizationPercentile;
///////

    eigenFeature->signatureAsVector.push_back((e1 - e2) / e1 / kLinearityMax);//linearity
    eigenFeature->signatureAsVector.push_back((e2 - e3) / e1 / kPlanarityMax);//planarity
    eigenFeature->signatureAsVector.push_back(e3 / e1 / kScatteringMax);//scattering
    eigenFeature->signatureAsVector.push_back(std::pow(e1 * e2 * e3, kOneThird) / kOmnivarianceMax);//omnivariance
    eigenFeature->signatureAsVector.push_back((e1 - e3) / e1 / kAnisotropyMax);//anisotropy
    eigenFeature->signatureAsVector.push_back(((e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3))) / kEigenEntropyMax);//eigen_entropy
    eigenFeature->signatureAsVector.push_back(e3 / sum_of_eigenvalues / kChangeOfCurvatureMax);//change_of_curvature

    pcl::PointXYZ point_min, point_max;

    pcl::getMinMax3D(*(segment->getSegmentCloud()), point_min, point_max);

    double diff_x, diff_y, diff_z;

    diff_x = point_max.x - point_min.x;
    diff_y = point_max.y - point_min.y;
    diff_z = point_max.z - point_min.z;

    if (diff_z < diff_x && diff_z < diff_y)
    {
        eigenFeature->signatureAsVector.push_back(0.2);//pointing_up
    }
    else
    {
        eigenFeature->signatureAsVector.push_back(0.0);//pointing_down
    }
}
