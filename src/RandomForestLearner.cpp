//
// Created by akhil on 03.05.17.
//


#include <RandomForestLearner.hpp>

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

void RandomForestLearner::trainMultiClass(const cv::Mat &trainFeatures, const cv::Mat &trainLabels, int numTrees){
    //RTrees
    CvRTParams trees_params;
    //float priors2[] = {0.5,0.5};
    int maxDepth = 60;
    trees_params = CvRTParams(
            maxDepth,			/* max_depth */
            3,				/* min_sample_count */
            0,									/* regression_accuracy */
            false,								/* use_surrogates */
            10,									/* max_categories */
            0,								/* priors */
            true,								/* calc_var_importance */
            0,									/* nactive_vars */
            numTrees,		/* max_num_of_trees_in_the_forest */
            0,									/* forest_accuracy */
            CV_TERMCRIT_ITER|CV_TERMCRIT_EPS
    );
    trees_params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, numTrees, 0);
    pcl::console::print_highlight("Learning trees with a max depth of %d, a max amount of %d trees, breaking with an error smaller than %f.",
                    trees_params.max_depth, trees_params.term_crit.max_iter, trees_params.term_crit.epsilon);

    poleTreeClassifier->clear();

    cout << "starting forest training ..." << endl;
    poleTreeClassifier->train(trainFeatures, CV_ROW_SAMPLE, trainLabels, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), trees_params);

    poleTreeClassifier->save("poleTreeClassifier.xml");
    cout << "forest training saved!" << endl;
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