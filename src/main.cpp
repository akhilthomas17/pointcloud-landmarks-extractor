#include <pole_detector.h>
int main(int argc, char const *argv[])
{
	if ( argc != 4 ) // argc should be 3 for correct execution
		cout<<"usage: "<< argv[0] <<" <path to pcd file> <Number of cuts> <Distance threshold cluster>\n";
	else {
		PCLPoleDetector* poleDetector = new PCLPoleDetector;
		poleDetector->algorithmLanda(argv[1], atof(argv[2]), atof(argv[3]));
	}

	return 0;
}