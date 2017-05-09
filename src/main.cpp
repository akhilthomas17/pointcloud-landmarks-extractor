#include <pole_detector.h>
int main(int argc, char const *argv[])
{
	if ( argc != 4 ) // argc should be 5 for correct execution
    {
        cout<<"usage: "<< argv[0] <<" <path to pcd file> <DON small scale> <mode>\n";
        pcl::console::print_highlight("[mode]\n0 : To use esf based features\n1 : To use eigen value based features\n2 : To use combination of above two\n");
    }

	else {
		PCLPoleDetector* poleDetector = new PCLPoleDetector;
		//string pathToPcd = "../results/don-optimization/no-filter/raw-don/don_0-5+5.pcd";
        int mode = atoi(argv[3]);
        /*
        string pathToDataFolder;
        switch (mode)
        {
            case 0: pathToDataFolder = "../data/esf_training/";
                break;
            case 1: pathToDataFolder = "../data/eigen_training/";
                break;
            case 2: pathToDataFolder = "../data/combined_training/";
                break;
        }
         */
        string pathToClassifier = "../data/rand_forest_multiclass_training/PoleTreeClassifier.xml";
        poleDetector->algorithmClassifierBased(argv[1], pathToClassifier, atof(argv[2]), mode);
		//poleDetector->algorithmFeatureDescriptorBased(argv[1], pathToDataFolder, atof(argv[2]), atof(argv[3]), mode);
	}

	return 0;
}
