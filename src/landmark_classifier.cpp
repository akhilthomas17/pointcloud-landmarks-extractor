#include <Master.hpp>
int main(int argc, char const *argv[])
{
	if ( argc != 4 ) // argc should be 6 for correct execution
    {
        cout<<"usage: "<< argv[0] <<" <path to input pcd file> <DON small radius> <DON cluster threshold>\n";
    }

	else {
		Master* master = new Master;
        int featureMode = 2; // To use combined features
        bool inputMode = false; // False to input raw pcd file
        bool classifierMode = true; // True to use random forest classifier
        double knnThreshold = 0; // Needed if KNN is used for classification
        if (!classifierMode)
            knnThreshold = 0.3;
        string pathToClassifier = "../data/classifier/PoleTreeClassifier.xml"; // relative path to the trained classifier
        master->runLandmarkClassifier(inputMode, argv[1], atof(argv[2]), atof(argv[3]),
                                      featureMode, classifierMode, pathToClassifier, knnThreshold);
	}

	return 0;
}
