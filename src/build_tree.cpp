#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/features/esf.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>

typedef std::pair<std::string, std::vector<float> > esf_model;

/** \brief Loads an n-D histogram file as a ESF signature
  * \param path the input file name
  * \param esf the resultant ESF model
  */
bool
loadHist (const boost::filesystem::path &path, esf_model &esf)
{
  // Cloud for storing the object.
  pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path.string (), *object) != 0)
  {
    return -1;
  }

  // Treat the ESF signature as a single Point Cloud
  pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
  esf.second.resize (640);

  // ESF estimation object.
  pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf_est;
  esf_est.setInputCloud(object);

  esf_est.compute(*descriptor);

  std::vector <pcl::PCLPointField> fields;
  int esf_idx;
  esf_idx = pcl::getFieldIndex (*descriptor, "esf", fields);

  for (size_t i = 0; i < fields[esf_idx].count; ++i)
  {
    esf.second[i] = descriptor->points[0].histogram[i];
  }
  esf.first = path.string ();
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
                   std::vector<esf_model> &models)
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
      loadFeatureModels (it->path (), extension, models);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      esf_model m;
      if (loadHist (base_dir / it->path ().filename (), m))
        models.push_back (m);
    }
  }
}

int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    PCL_ERROR ("Need at least two parameters! Syntax is: %s [model_directory] [options]\n", argv[0]);
    return (-1);
  }

  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

  std::string kdtree_idx_file_name = "kdtree.idx";
  std::string training_data_h5_file_name = "training_data.h5";
  std::string training_data_list_file_name = "training_data.list";

  std::vector<esf_model> models;

  // Load the model histograms
  loadFeatureModels (argv[1], extension, models);
  pcl::console::print_highlight ("Loaded %d VFH models. Creating training data %s/%s.\n", 
      (int)models.size (), training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());

  // Convert data into FLANN format
  flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

  for (size_t i = 0; i < data.rows; ++i)
    for (size_t j = 0; j < data.cols; ++j)
      data[i][j] = models[i].second[j];

  // Save data to disk (list of models)
  flann::save_to_file (data, training_data_h5_file_name, "training_data");
  std::ofstream fs;
  fs.open (training_data_list_file_name.c_str ());
  for (size_t i = 0; i < models.size (); ++i)
    fs << models[i].first << "\n";
  fs.close ();
 
  // Build the tree index and save it to disk
  pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
  //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
  flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
  index.buildIndex ();
  index.save (kdtree_idx_file_name);
  delete[] data.ptr ();

  return (0);
}