#include <Master.hpp>
int main(int argc, char const *argv[])
{
	if ( argc != 6 ) // argc should be 6 for correct execution
    {
        cout<<"usage: "<< argv[0] <<" <input mode> <path to pcd file> <classifier mode> "
                "<path to classifier model> <DON cluster threshold>\n";
        pcl::console::print_highlight("[input mode]\n1 : Input DON file"
                                              "\n0 : Input raw PCD file\n");
        pcl::console::print_highlight("[classifier mode]\n1 : Random forest classifier"
                                              "\n0 : KNN\n");
    }

	else {
		Master* master = new Master;
        int featureMode = 2; // To use combined features
        bool inputMode = (bool) atoi(argv[1]);
        bool classifierMode = (bool) atoi(argv[3]);
        double knnThreshold = 0;
        if (!classifierMode)
            knnThreshold = 0.3;
        string pathToClassifier = argv[4];
        master->runLandmarkClassifier(inputMode, argv[2], atof(argv[5]), featureMode,
                                            classifierMode, pathToClassifier, knnThreshold);
	}

	return 0;
}
