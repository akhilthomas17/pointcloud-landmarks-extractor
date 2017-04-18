#include <pole_detector.h>
int main(int argc, char const *argv[])
{
	if ( argc != 4 ) // argc should be 4 for correct execution
		cout<<"usage: "<< argv[0] <<" <path to pcd file> <DON small scale> <kdTree Threshold> \n";
	else {
		PCLPoleDetector* poleDetector = new PCLPoleDetector;
		//string pathToPcd = "../results/don-optimization/no-filter/raw-don/don_0-5+5.pcd";
		string pathToDataFolder = "../data/esf_training/";
		poleDetector->algorithmFeatureDescriptorBased(argv[1], pathToDataFolder, atof(argv[2]), atof(argv[3]));
	}

	return 0;
}
