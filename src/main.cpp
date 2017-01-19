#include <pole_detector.h>
int main(int argc, char const *argv[])
{
	if ( argc != 2 ) // argc should be 2 for correct execution
		cout<<"usage: "<< argv[0] <<" <path to pcd file>\n";
	else {
		PCLPoleDetector* poleDetector = new PCLPoleDetector;
		poleDetector->readPCD(argv[1]);
		poleDetector->preProcessor(0.1, 0.6);
		//poleDetector->removeGroundPoints_height(-1.5);
		//poleDetector->writePCD("output_pcd.pcd");


/*  Testing MinMax3D function
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  		cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud);
	  	pcl::PointXYZ minPt, maxPt;
	  	pcl::getMinMax3D (*cloud, minPt, maxPt);
	  	std::cout << "Max x: " << maxPt.x << std::endl;
	  	std::cout << "Max y: " << maxPt.y << std::endl;
	  	std::cout << "Max z: " << maxPt.z << std::endl;
	  	std::cout << "Min x: " << minPt.x << std::endl;
	  	std::cout << "Min y: " << minPt.y << std::endl;
	  	std::cout << "Min z: " << minPt.z << std::endl;
*/
	}

	return 0;
}