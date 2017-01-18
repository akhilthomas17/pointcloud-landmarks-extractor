#include <pole_detector.h>
int main(int argc, char const *argv[])
{
	if ( argc != 2 ) // argc should be 2 for correct execution
		cout<<"usage: "<< argv[0] <<" <path to pcd file>\n";
	else {
		PCLPoleDetector* poleDetector = new PCLPoleDetector;
		poleDetector->readPCD(argv[1]);
		poleDetector->removeGroundPoints_height(-1.5);
		poleDetector->writePCD("output_pcd.pcd");
	}

	return 0;
}