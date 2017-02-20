#include <pole_detector.h>
int main(int argc, char const *argv[])
{
	if ( argc != 7 ) // argc should be 7 for correct execution
		cout<<"usage: "<< argv[0] <<" <path to pcd file> <maxDistanceStitches> <minPoleHeight> <maxDistanceTrees> <DON small scale> <DON large scale> \n";
	else {
		PCLPoleDetector* poleDetector = new PCLPoleDetector;
		poleDetector->algorithmSingleCut(argv[1], atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]));
	}

	return 0;
}