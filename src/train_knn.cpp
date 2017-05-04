#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>
#include <FeatureDescriptor.h>


/** \brief Opens the file and compute a feature signature as per the mode
  * \param path the input file name
  * \param feature the resultant Feature
  */
bool
computeFeature (const boost::filesystem::path &path, Feature &feature, int mode)
{
    FeatureDescriptor descriptor;

    // Cloud for storing the object.
    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path.string (), *object) != 0)
    {
        return -1;
    }

    switch (mode){
        case 0:
            descriptor.describeEsfFeature(object,&feature);
            break;
        case 1: {
            Segment segment;
            segment.setSegmentCloud(object);
            segment.computeCentroid();
            descriptor.describeEigenFeature(&segment,&feature);
            break;
        }
        case 2: {
            descriptor.describeEsfFeature(object, &feature);
            Feature eigVal;
            Segment segment;
            segment.setSegmentCloud(object);
            segment.computeCentroid();
            descriptor.describeEigenFeature(&segment, &eigVal);
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
                   std::vector<Feature> &models, int mode)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
      loadFeatureModels (it->path (), extension, models, mode);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      Feature m;
      if (computeFeature (base_dir / it->path ().filename (), m, mode))
        models.push_back (m);
    }
  }
}

int
main (int argc, char** argv)
{
    // Command line parsing and getting the operation mode (Which feature descriptor to be used)
    int mode = 0;
    if (argc < 2)
    {
      PCL_ERROR ("Need at least three parameters! Syntax is: %s [model_directory] [options]\n", argv[0]);
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
    }
    pcl::console::print_highlight("Mode selected is %d\n", mode);


    string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    string kdtree_idx_file_name = std::string(argv[1]) + "kdtree.idx";
    string training_data_h5_file_name = std::string(argv[1]) + "training_data.h5";
    string training_data_list_file_name = std::string(argv[1]) + "training_data.list";
    string mode_file_name = string(argv[1]) + "mode.txt";

    std::ofstream mode_file(mode_file_name.c_str(), ios::out | ios::trunc);
    mode_file << "Mode: " << mode << endl;


    std::vector<Feature> models;

    // Load the model histograms
    loadFeatureModels (argv[1], extension, models, mode);
    pcl::console::print_highlight ("Loaded %d VFH models. Creating training data %s/%s.\n",
      (int)models.size (), training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());

    // Convert data into FLANN format
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

    return (0);
}
