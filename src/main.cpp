#include <pole_detector.h>
int main(int argc, char const *argv[])
{
	if ( argc != 4 ) // argc should be 5 for correct execution
		cout<<"usage: "<< argv[0] <<" <path to pcd file> <DON small scale> <DON large scale>\n";
	else {
		PCLPoleDetector* poleDetector = new PCLPoleDetector;
		poleDetector->algorithmSingleCut(argv[1], atof(argv[2]), atof(argv[3]));
	}

	return 0;
}