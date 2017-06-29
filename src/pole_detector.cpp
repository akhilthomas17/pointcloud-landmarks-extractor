//
// Created by akhil
//

#include <Master.hpp>
int main(int argc, char const *argv[])
{
    if ( argc != 4 ) // argc should be 5 for correct execution
    {
        cout<<"usage: "<< argv[0] <<" <input mode> <path to pcd file> <DON cluster threshold>\n";
        pcl::console::print_highlight("[input mode]\n1 : Input DON file"
                                              "\n0 : Input raw PCD file\n");
    }

	else {
		Master* master = new Master;
        bool inputMode = (bool) atoi(argv[1]);
        double donThreshold = atof(argv[3]);
        double maxDistanceStitches = 2;
        double xyBoundThreshold = 1;
        double minPoleHeight = 3;
        master->runPoleDetector(inputMode, argv[2], donThreshold, maxDistanceStitches,
                                xyBoundThreshold, minPoleHeight);
	}

	return 0;
}