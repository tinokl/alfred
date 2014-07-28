#include <alfred_occipital_lobe/alfred_occipital_lobe_node.h>

#define TRAINING_SAMPLES 3050       //Number of samples in training dataset
#define ATTRIBUTES 256              //Number of pixels per sample.16X16
#define TEST_SAMPLES 1170           //Number of samples in test dataset
#define CLASSES 10                  //Number of distinct labels.

namespace alfred_occipital_lobe
{

AlfredOccipitalLobe::AlfredOccipitalLobe()
: it_(nh_)
{

}

AlfredOccipitalLobe::~AlfredOccipitalLobe()
{
    //delete transformation_;
   cv::destroyWindow(OPENCV_WINDOW);
}


void AlfredOccipitalLobe::init()
{
    
    ros::NodeHandle nh("~");

    nh.getParam("topic",topic_);
    nh.getParam("train",train_);
    nh.getParam("debug",debug_);

    //ROS_ERROR_STREAM("topic is this: " << topic_);

    if (train_)
        image_sub_ = it_.subscribe(topic_, 1, &AlfredOccipitalLobe::detection, this);
    //else
    //    image_sub_ = it_.subscribe(topic_, 1, &AlfredOccipitalLobe::training, this);

    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);

    /*

    if(!debug)
    {
       // transformation_ = new TransformationWithRaycasting(use_transform, target_frame);
       //  transformation_->calcDataFromFieldOfView(fov_x, fov_y);
    }
    */
}

void AlfredOccipitalLobe::detection(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // encoding is Grayscale CV_8UC1
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  cv::Mat img = cv_ptr->image;//cv::imread(imagePath,0);
  cv::Mat output;

  //applying gaussian blur to remove any noise
  cv::GaussianBlur(img,output,cv::Size(5,5),0);
  //thresholding to get a binary image
  cv::threshold(output,output,50,255,0);

  //cropping the image. IS DANGEROUS!
  //cropImage(output,output);

  //declaring mat to hold the scaled down image
  cv::Mat scaled_down_image(16,16,CV_8U,cv::Scalar(0));
  //reducing the image dimension to 16X16

  scaleDownImage(output,scaled_down_image);

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, output);
  cv::waitKey(3);

  //declaring array to hold the pixel values in the memory before it written into file
  int pixel_value_array[256];
  //reading the pixel values.

  convertToPixelValueArray(scaled_down_image,pixel_value_array);

  cv::Mat input(1,ATTRIBUTES,CV_32F);
  for (int i = 0; i <= ATTRIBUTES; i++)
      input.at<float>(i) = (float)pixel_value_array[i];

  //cv::Size s = input.size();
  //int rows = s.height;
  //int cols = s.width;
  //ROS_ERROR("input rows: %d, cols: %d",rows,cols);

  //classification
  classification(input);

  /*
  // Draw an example circle on the video stream
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));


  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
  */
}

void AlfredOccipitalLobe::cropImage(cv::Mat &originalImage,cv::Mat &croppedImage)
{
    int row = originalImage.rows;
    int col = originalImage.cols;
    int tlx,tly,bry,brx;//t=top r=right b=bottom l=left
    tlx=tly=bry=brx=0;
    float suml=0;
    float sumr=0;
    int flag=0;

    /**************************top edge***********************/
    for(int x=1;x<row;x++)
    {
        for(int y = 0;y<col;y++)
        {
            if(originalImage.at<uchar>(x,y)==0)
            {

                flag=1;
                tly=x;
                break;
            }

        }
        if(flag==1)
        {
            flag=0;
            break;
        }

    }
    /*******************bottom edge***********************************/
    for(int x=row-1;x>0;x--)
    {
        for(int y = 0;y<col;y++)
        {
            if(originalImage.at<uchar>(x,y)==0)
            {

                flag=1;
                bry=x;
                break;
            }

        }
        if(flag==1)
        {
            flag=0;
            break;
        }

    }
    /*************************left edge*******************************/

    for(int y=0;y<col;y++)
    {
        for(int x = 0;x<row;x++)
        {
            if(originalImage.at<uchar>(x,y)==0)
            {

                flag=1;
                tlx=y;
                break;
            }

        }
        if(flag==1)
        {
            flag=0;
            break;
        }
    }

    /**********************right edge***********************************/

    for(int y=col-1;y>0;y--)
    {
        for(int x = 0;x<row;x++)
        {
            if(originalImage.at<uchar>(x,y)==0)
            {

                flag=1;
                brx= y;
                break;
            }

        }
        if(flag==1)
        {
            flag=0;
            break;
        }
    }
    int width = brx-tlx; //′s and store
    int height = bry-tly;
    cv::Mat crop(originalImage,cv::Rect(tlx,tly,brx-tlx,bry-tly));
    croppedImage= crop.clone();

}

void AlfredOccipitalLobe::scaleDownImage(cv::Mat &originalImg,cv::Mat &scaledDownImage)
{
    for(int x=0;x<16;x++)
    {
        for(int y=0;y<16 ;y++)
        {
            int yd =ceil((float)(y*originalImg.cols/16));
            int xd = ceil((float)(x*originalImg.rows/16));
            scaledDownImage.at<uchar>(x,y) = originalImg.at<uchar>(xd,yd);

        }
    }
}

void AlfredOccipitalLobe::convertToPixelValueArray(cv::Mat &img,int pixelarray[])
{
    int i =0;
    for(int x=0;x<16;x++)
    {
        for(int y=0;y<16;y++)
        {
            pixelarray[i]=(img.at<uchar>(x,y)==255)?1:0;
            i++;
        }
    }
}

void read_dataset(char *filename, cv::Mat &data, cv::Mat &classes,  int total_samples)
{
    // This function will read the csv files(training and test dataset) and convert them
    // into two matrices. classes matrix have 10 columns, one column for each class label. If the label of nth row in data matrix
    // is, lets say 5 then the value of classes[n][5] = 1.

    int label;
    float pixelvalue;
    //open the file
    FILE* inputfile = fopen( filename, "r" );

    //read each row of the csv file
   for(int row = 0; row < total_samples; row++)
   {
       //for each attribute in the row
     for(int col = 0; col <=ATTRIBUTES; col++)
        {
            //if its the pixel value.
            if (col < ATTRIBUTES){

                fscanf(inputfile, "%f,", &pixelvalue);
                data.at<float>(row,col) = pixelvalue;

            }//if its the label
            else if (col == ATTRIBUTES){
                //make the value of label column in that row as 1.
                fscanf(inputfile, "%i", &label);
                classes.at<float>(row,label) = 1.0;

            }
        }
    }

    fclose(inputfile);

}

void AlfredOccipitalLobe::classification(cv::Mat input)
{
    //read the model from the XML file and create the neural network.
    CvANN_MLP nnetwork;
    CvFileStorage* storage = cvOpenFileStorage( "/home/kl/hydro/alfred_ws/src/alfred/alfred_occipital_lobe/trained/numbers.xml", 0, CV_STORAGE_READ );
    CvFileNode *n = cvGetFileNodeByName(storage,0,"DigitOCR");
    nnetwork.read(storage,n);
    cvReleaseFileStorage(&storage);

    //your code here
    // ...Generate cv::Mat data(1,ATTRIBUTES,CV_32S) which will contain the pixel
    // ... data for the digit to be recognized

    int maxIndex = 0;
    cv::Mat classOut(1,CLASSES,CV_32F);
    //prediction
    nnetwork.predict(input, classOut);
    float value;
    float maxValue=classOut.at<float>(0,0);
    for(int index=1;index<CLASSES;index++)
    {
        value = classOut.at<float>(0,index);
        if(value>maxValue)
        {
            maxValue = value;
            maxIndex=index;
        }
    }
    ROS_ERROR("PREDICTED: %d, with value: %f",maxIndex, maxValue);
    //maxIndex is the predicted class.
}

/*
void AlfredOccipitalLobe::training(cv::Mat input)
{
//matrix to hold the training sample
cv::Mat training_set(TRAINING_SAMPLES,ATTRIBUTES,CV_32F);
//matrix to hold the labels of each taining sample
cv::Mat training_set_classifications(TRAINING_SAMPLES, CLASSES, CV_32F);
//matric to hold the test samples
cv::Mat test_set(TEST_SAMPLES,ATTRIBUTES,CV_32F);
//matrix to hold the test labels.
cv::Mat test_set_classifications(TEST_SAMPLES,CLASSES,CV_32F);

//
cv::Mat classificationResult(1, CLASSES, CV_32F);
//load the training and test data sets.
read_dataset("C:\\Users\\NITHIN\\Desktop\\OCR\\NN\\OCR\\training_dataset.txt", training_set, training_set_classifications, TRAINING_SAMPLES);
read_dataset("C:\\Users\\NITHIN\\Desktop\\OCR\\NN\\OCR\\test_dataset.txt", test_set, test_set_classifications, TEST_SAMPLES);

    // define the structure for the neural network (MLP)
    // The neural network has 3 layers.
    // - one input node per attribute in a sample so 256 input nodes
    // - 16 hidden nodes
    // - 10 output node, one for each class.

    cv::Mat layers(3,1,CV_32S);
    layers.at<int>(0,0) = ATTRIBUTES;//input layer
    layers.at<int>(1,0)=16;//hidden layer
    layers.at<int>(2,0) =CLASSES;//output layer

    //create the neural network.
    //for more details check http://docs.opencv.org/modules/ml/doc/neural_networks.html
    CvANN_MLP nnetwork(layers, CvANN_MLP::SIGMOID_SYM,0.6,1);

    CvANN_MLP_TrainParams params(

                                    // terminate the training after either 1000
                                    // iterations or a very small change in the
                                    // network wieghts below the specified value
                                    cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 0.000001),
                                    // use backpropogation for training
                                    CvANN_MLP_TrainParams::BACKPROP,
                                    // co-efficents for backpropogation training
                                    // recommended values taken from http://docs.opencv.org/modules/ml/doc/neural_networks.html#cvann-mlp-trainparams
                                    0.1,
                                    0.1);

    // train the neural network (using training data)

    printf( "\nUsing training dataset\n");
    int iterations = nnetwork.train(training_set, training_set_classifications,cv::Mat(),cv::Mat(),params);
    printf( "Training iterations: %i\n\n", iterations);

    // Save the model generated into an xml file.
    CvFileStorage* storage = cvOpenFileStorage( "C:\\Users\\NITHIN\\Desktop\\OCR\\NN\\OCR1\\param.xml", 0, CV_STORAGE_WRITE );
    nnetwork.write(storage,"DigitOCR");
    cvReleaseFileStorage(&storage);

    // Test the generated model with the test samples.
    cv::Mat test_sample;
    //count of correct classifications
    int correct_class = 0;
    //count of wrong classifications
    int wrong_class = 0;

    //classification matrix gives the count of classes to which the samples were classified.
    int classification_matrix[CLASSES][CLASSES]={{}};

    // for each sample in the test set.
    for (int tsample = 0; tsample < TEST_SAMPLES; tsample++) {

        // extract the sample

        test_sample = test_set.row(tsample);

        //try to predict its class

        nnetwork.predict(test_sample, classificationResult);
        //The classification result matrix holds weightage  of each class.
        //we take the class with the highest weightage as the resultant class

        // find the class with maximum weightage.
        int maxIndex = 0;
        float value=0.0f;
        float maxValue=classificationResult.at<float>(0,0);
        for(int index=1;index<CLASSES;index++)
        {   value = classificationResult.at<float>(0,index);
            if(value>maxValue)
            {   maxValue = value;
                maxIndex=index;

            }
        }

        printf("Testing Sample %i -> class result (digit %d)\n", tsample, maxIndex);

        //Now compare the predicted class to the actural class. if the prediction is correct then\
        //test_set_classifications[tsample][ maxIndex] should be 1.
        //if the classification is wrong, note that.
        if (test_set_classifications.at<float>(tsample, maxIndex)!=1.0f)
        {
            // if they differ more than floating point error => wrong class

            wrong_class++;

            //find the actual label 'class_index'
            for(int class_index=0;class_index<CLASSES;class_index++)
            {
                if(test_set_classifications.at<float>(tsample, class_index)==1.0f)
                {

                    classification_matrix[class_index][maxIndex]++;// A class_index sample was wrongly classified as maxindex.
                    break;
                }
            }

        } else {

            // otherwise correct

            correct_class++;
            classification_matrix[maxIndex][maxIndex]++;
        }
    }
    /*
    printf( "\nResults on the testing dataset\n"
    "\tCorrect classification: %d (%g%%)\n"
    "\tWrong classifications: %d (%g%%)\n",
    correct_class, (double) correct_class*100/TEST_SAMPLES,
    wrong_class, (double) wrong_class*100/TEST_SAMPLES);
    cout<<"   ";
    for (int i = 0; i < CLASSES; i++)
    {
        cout<< i<<"\t";
    }
    cout<<"\n";
    for(int row=0;row<CLASSES;row++)
    {cout<<row<<"  ";
        for(int col=0;col<CLASSES;col++)
        {
            cout<<classification_matrix[row][col]<<"\t";
        }
        cout<<"\n";
    }
*/
}



int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "alfred_occipital_lobe" );
        alfred_occipital_lobe::AlfredOccipitalLobe alfred_occipital_lobe;
        alfred_occipital_lobe.init();
        ros::spin();
    }
    catch( ... )
    {
        ROS_ERROR_NAMED("alfred_occipital_lobe","Unhandled exception!");
        return -1;
    }

    return 0;
}
