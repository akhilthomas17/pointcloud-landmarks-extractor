#include <pole_detector.h>
int main(int argc, char const *argv[])
{
	if ( argc != 3 ) // argc should be 3 for correct execution
		cout<<"usage: "<< argv[0] <<" <path to pcd file> <Distance threshold plane removal>\n";
	else {
		PCLPoleDetector* poleDetector = new PCLPoleDetector;
		poleDetector->algorithmLanda(argv[1], atof(argv[2]));
	}

	return 0;
}