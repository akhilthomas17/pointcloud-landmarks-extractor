//
// Created by akhil on 03.05.17.
//


#include <Classifier/RandomForestLearner.hpp>

RandomForestLearner::RandomForestLearner() {
    treeClassifier = new CvRTrees;
    poleClassifier = new CvRTrees;
    poleTreeClassifier = new CvRTrees;
}

RandomForestLearner::~RandomForestLearner() {
    delete poleClassifier;
    delete treeClassifier;
    delete poleTreeClassifier;
}

void RandomForestLearner::trainMultiClass(const cv::Mat &trainFeatures, const cv::Mat &trainLabels, CvRTParams trees_params ){
    //RTrees
    pcl::console::print_highlight("Learning trees with a max depth of %d, a max amount of %d trees, breaking with an error smaller than %f.",
                    trees_params.max_depth, trees_params.term_crit.max_iter, trees_params.term_crit.epsilon);

    poleTreeClassifier->clear();

    cout << "starting forest training ..." << endl;
    poleTreeClassifier->train(trainFeatures, CV_ROW_SAMPLE, trainLabels, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), trees_params);

//	trees->load("trees.xml");
    cout << "varImportance:" << endl << poleTreeClassifier->getVarImportance() << endl;
    ofstream sss("PLOTS/varImportance.gnuplot");
    sss << "set terminal pdf size 4,4; set output 'PLOTS/varImportance.pdf'; plot " << endl;
    for(int w=0; w < poleTreeClassifier->getVarImportance().cols; w++){
        sss << "	" << poleTreeClassifier->getVarImportance().at<float>(w) << endl;
        if(((w+1)==(poleTreeClassifier->getVarImportance().cols/3.0)) ||
                ((w+1)==(2*poleTreeClassifier->getVarImportance().cols/3.0)) ||
                ((w+1)==(3*poleTreeClassifier->getVarImportance().cols/3.0)))
            sss << "EOF" << endl;
    }
    stringstream ss;
    ss << "gnuplot PLOTS/varImportance.gnuplot";
    if (system(ss.str().c_str()) < 0)
        cout << "wasn't able to call gnuplot" << std::endl;
}

void RandomForestLearner::saveClassifier(string pathToClassifier){
    poleTreeClassifier->save(pathToClassifier.c_str());
    cout << "Forest classifier saved in " << pathToClassifier << endl;
}

void RandomForestLearner::loadClassifier(string pathToClassifier){
    poleTreeClassifier->load(pathToClassifier.c_str());
}

void RandomForestLearner::predictClass(Feature candidate, string& predictedClass) {
    cv::Mat featureMat;
    cv::Mat(candidate.signatureAsVector).copyTo(featureMat);
    int label = (int) poleTreeClassifier->predict(featureMat);
    switch (label){
        case 0:
            predictedClass = "Pole";
            break;
        case 1:
            predictedClass = "Tree";
            break;
        case 2:
            predictedClass = "None";
            break;
    }
}