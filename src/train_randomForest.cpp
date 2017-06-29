#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <Classifier/FeatureExtractor.hpp>
#include <Libs/Structures.hpp>
#include <Classifier/RandomForestLearner.hpp>

/** \brief Opens the pcd file and compute a feature signature as per the mode
  * \param path the input file name
  * \param feature the resultant Feature
  * \param mode the feature descriptor used
  */
bool
computeFeature (const boost::filesystem::path &path, Feature &feature, int mode)
{
    FeatureExtractor descriptor;

    // Cloud for storing the object.
    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path.string (), *object) != 0)
    {
        return -1;
    }

    switch (mode){
        case 0:
            descriptor.extractEsfFeature(object, &feature);
            break;
        case 1: {
            Segment segment;
            segment.setSegmentCloud(object);
            segment.computeCentroid();
            descriptor.extractEigenFeature(&segment, &feature);
            break;
        }
        case 2: {
            descriptor.extractEsfFeature(object, &feature);
            Feature eigVal;
            Segment segment;
            segment.setSegmentCloud(object);
            segment.computeCentroid();
            descriptor.extractEigenFeature(&segment, &eigVal);
            feature += eigVal;
            break;
        }
    }

    feature.name = path.string ();
    return (true);
}

/** \brief Load a set of ESF features that will act as the model (training data)
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param extension the file extension of files to compute the ESF features
  * \param models the resultant vector of histogram models
  */
void
loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension,
                   cv::Mat* _features, cv::Mat* _labels, int mode)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

    vector<int> labels;
    _features->release();
    _labels->release();


    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)_labels->rows);
      loadFeatureModels (it->path (), extension, _features, _labels, mode);
    }

      if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      Feature m;
      if (computeFeature (base_dir / it->path ().filename (), m, mode)) {
          //cerr << _features->rows<<endl;
          cv::Mat feature(648, 1, CV_32F);
          cv::Mat(m.signatureAsVector).copyTo(feature);
          _features->push_back(feature.reshape(0,1));

          if (m.name.find("Pole") != -1) {
              labels.push_back(0);
          } else if (m.name.find("Tree") != -1) {
              labels.push_back(1);
          } else
              labels.push_back(2);

      }
    }
  }
    cv::Mat(labels).copyTo(*_labels);
}

int
main (int argc, char** argv)
{
    // Command line parsing and getting the operation mode (Which feature descriptor to be used)
    int mode = 2;
    int maxDepth = 60;
    int numTrees = 100;
    if (argc < 2)
    {
      PCL_ERROR ("Need at least two parameters! Syntax is: %s [model_directory] [options] [max_depth] [num_trees]\n", argv[0]);
      pcl::console::print_highlight("[options] \n--eigVal : To use eigen value based features\n--esf : To use esf based features\n--combined : To use combination of above two\n");
      return (-1);
    }
    else if(argc>2)
    {
        if (strcmp(argv[2], "--esf")==0)
            mode = 0;
        else if (strcmp(argv[2], "--eigVal")==0)
            mode = 1;
        else if (strcmp(argv[2], "--combined")==0)
            mode = 2;
        if (argc > 3){
            maxDepth = atoi(argv[3]);
            numTrees = atoi(argv[4]);
        }
    }
    pcl::console::print_highlight("Mode selected is %d\n", mode);


    string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    string mode_file_name = string(argv[1]) + "mode.txt";
    string classifier_file = string(argv[1]) + "PoleTreeClassifier.xml";

    std::ofstream mode_file(mode_file_name.c_str(), ios::out | ios::trunc);
    mode_file << "Mode: " << mode << endl;


    cv::Mat features;
    cv::Mat labels;

    // Load the model histograms
    loadFeatureModels (argv[1], extension, &features, &labels, mode);
    pcl::console::print_highlight("Feature rows: %d, Label rows: %d \n", features.rows, labels.rows);
    pcl::console::print_highlight ("Loaded %d clusters. Training the classifier.\n", features.rows);

    RandomForestLearner classifier;
    CvRTParams trees_params;
    //float priors2[] = {0.5,0.5};

    trees_params = CvRTParams(
            maxDepth,			/* max_depth */
            2,				/* min_sample_count */
            0,									/* regression_accuracy */
            false,								/* use_surrogates */
            15,									/* max_categories */
            0,								/* priors */
            true,								/* calc_var_importance */
            50,									/* nactive_vars ääööää*/
            numTrees,		/* max_num_of_trees_in_the_forest */
            0,									/* forest_accuracy */
            CV_TERMCRIT_ITER|CV_TERMCRIT_EPS
    );
    trees_params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, numTrees, 0);
    classifier.trainMultiClass(features, labels, trees_params);
    classifier.saveClassifier(classifier_file);
    features.release();
    labels.release();



    // Convert data into FLANN format
    /*
    flann::Matrix<float> data (new float[models.size () * models[0].signatureAsVector.size ()], models.size (), models[0].signatureAsVector.size ());

    for (size_t i = 0; i < data.rows; ++i)
    for (size_t j = 0; j < data.cols; ++j)
      data[i][j] = models[i].signatureAsVector[j];

    // Save data to disk (list of models)
    flann::save_to_file (data, training_data_h5_file_name, "training_data");
    std::ofstream fs;
    fs.open (training_data_list_file_name.c_str ());
    for (size_t i = 0; i < models.size (); ++i)
    fs << models[i].name << "\n";
    fs.close ();


    // Build the tree index and save it to disk
    pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
    //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
    index.buildIndex ();
    index.save (kdtree_idx_file_name);
    delete[] data.ptr ();
     */

    return (0);
}
