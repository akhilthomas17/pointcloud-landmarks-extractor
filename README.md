# landmark-classifier tool

## Running instructions for landmark classifier
./landmark_classifier path to input pcd file DON small radius DON cluster threshold


###path to input pcd file
Path to the input map or scan (currently supports only ".pcd" extension)

###DON small radius
Small radius for the DON calculation.
0.4 or 0.5 would be a good choice!
Tip: Larger the radius, larger objects are clustered better!

###DON cluster threshold
Cluster threshold for DON based clustering.
0.35 is the default one when 0.5 small radius is used!