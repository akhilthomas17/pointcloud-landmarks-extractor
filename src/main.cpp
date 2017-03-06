#include <pole_detector.h>
int main(int argc, char const *argv[])
{
	if ( argc != 6 ) // argc should be 6 for correct execution
		cout<<"usage: "<< argv[0] <<" <path to pcd file> <xy bound> <maxDistanceStitches> <minPoleHeight> <DON small scale> \n";
	else {
		PCLPoleDetector* poleDetector = new PCLPoleDetector;
		poleDetector->algorithmSingleCut(argv[1], atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]));
	}

	return 0;
}