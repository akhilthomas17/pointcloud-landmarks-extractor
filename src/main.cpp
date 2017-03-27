#include <pole_detector.h>
int main(int argc, char const *argv[])
{
	if ( argc != 4 ) // argc should be 4 for correct execution
		cout<<"usage: "<< argv[0] <<" <path to pcd file> <maxDistanceStitches> <DON small scale> \n";
	else {
		PCLPoleDetector* poleDetector = new PCLPoleDetector;
		poleDetector->buildRefClusters(argv[1], atof(argv[2]), atof(argv[3]));
	}

	return 0;
}