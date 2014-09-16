// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <opencv2/opencv.hpp>

#include <Utils/GetPot>
#include <PlaceMatching/DBoWMatcher/DBoW2/DBoW2.h>
#include <PlaceMatching/DBoWMatcher/DUtils/DUtils.h>
#include <PlaceMatching/DBoWMatcher/DUtilsCV/DUtilsCV.h>
#include "../FMat8UBinary.h"
#include "../FMat32F.h"

const char* g_usage =
    "USAGE: DBoWMatcher-createvoc <options>\n"
    "DBoWMatcher vocabulary creator"
    "\n"
    "\t--input_dir\tDirectory of training image files, or filename of a text "
    "file with the paths to the images\n"
    "\t--detector\tName of feature detector (as supported by OpenCV)\n"
    "\t--descriptor\tName of feature descriptor (as supported by OpenCV)\n"
    "\t--k\tBranching factor k\n"
    "\t--L\tDepth levels L\n"
    "\t--ouput_dir\tDirectory where vocabulary is stored\n"
    "\n"
    "This will create a vocabulary of up to k^L words of the given feature.\n"
    "If only detector or descriptor is provided, the same is set to the "
    "other if possible.\n\n";

struct tParameters
{
  // params given by the user
  std::string input_dir, output_dir, detector, descriptor;
  int k, L;

  // params set by the app
  std::string output_filename; // filename of vocabulary
  bool binary_feature;         // feature is binary? float otherwise
};

/// Get and check application parameters. It exists if error
void getParameters(GetPot& cl, tParameters& params);

/// Get training image filenames
void getImageNames(const tParameters& params, std::vector<std::string> &files);

/// Runs the feature extration and vocabulary creation
template<class TDescriptor, class F>
void run(const tParameters& params, const std::vector<std::string>& image_files);

int main(int argc, char** argv)
{
  GetPot cl(argc, argv);
  if (cl.search("--help")) {
    std::cout << g_usage << std::endl;
    exit(-1);
  }

  // get params
  tParameters params;
  getParameters(cl, params);

  // load images
  std::vector<std::string> image_files;
  getImageNames(params, image_files);

  if(image_files.empty())
  {
    std::cout << "No training files found in input directory: " <<
                 params.input_dir << std::endl;
    return 0;
  }

  // ok, go
  // The DBoW2 format requires to specify as template parameters the
  // type of a single descriptor and the name of a virtual class that
  // can operate on this descriptor type.
  // We will use a cv::Mat vector of one row for each individual descriptor
  // (which can be CV_32S for binary descriptors packed as ints, or
  // CV_32F for float descriptors)
  if(params.binary_feature)
    run<cv::Mat, FMat8UBinary>(params, image_files);
  else
    run<cv::Mat, FMat32F>(params, image_files);
}

template<class TDescriptor, class F>
void run(const tParameters &params, const std::vector<std::string>& image_files)
{
  std::cout << "Creating vocabulary from " << params.detector << " points "
               "with " << params.descriptor << " descriptors" << std::endl;

  // get descriptors
  cv::Ptr<cv::FeatureDetector> detector =
      cv::FeatureDetector::create(params.detector);
  cv::Ptr<cv::DescriptorExtractor> extractor =
      cv::DescriptorExtractor::create(params.descriptor);

  std::vector<std::vector<TDescriptor> > all_descs;
  all_descs.reserve(image_files.size());

  std::cout << "Reading images";
  size_t counter = 0;
  for(auto file : image_files)
  {
    std::cout << "." << std::flush;
    cv::Mat im = cv::imread(file, false);

    if(im.empty())
      std::cout << std::endl << "Could not read file " << file << " ";
    else
    {
      std::vector<cv::KeyPoint> keys;
      cv::Mat descs;
      detector->detect(im, keys);
      extractor->compute(im, keys, descs);

      // convert into DBoW2 format (this assumes TDescriptor is cv::Mat)
      all_descs.push_back(std::vector<TDescriptor>());
      for(int r = 0; r < descs.rows; ++r)
        all_descs.back().push_back(descs.row(r));

      counter += all_descs.back().size();
    }
  } // for each file
  std::cout << std::endl << counter << " descriptors got from "
            << all_descs.size() << " images" << std::endl;

  // create voc
  std::cout << "Creating vocabulary..." << std::endl;
  DBoW2::TemplatedVocabulary<TDescriptor, F> voc;
  voc.create(all_descs, params.k, params.L);

  // save
  std::cout << "Saving..." << std::endl;
  voc.save(params.output_filename);

  std::cout << "Done" << std::endl;
}

void getParameters(GetPot& cl, tParameters& params)
{
  params.input_dir = cl.follow("", "--input_dir");
  params.output_dir = cl.follow(".", "--output_dir");
  params.detector = cl.follow("", "--detector");
  params.descriptor = cl.follow("", "--descriptor");
  params.k = cl.follow(0, "--k");
  params.L = cl.follow(0, "--L");

  if(params.k <= 0 || params.L <= 0)
  {
    std::cout << "Parameters k and L must be greater than 0" << std::endl;
    exit(-2);
  }

  if(params.detector.empty() && params.descriptor.empty())
  {
    std::cout << "At least a detector or descriptor name must be given"
                 << std::endl;
    exit(-3);
  }

  if(params.detector.empty() && !params.descriptor.empty())
  {
    params.detector = params.descriptor;
  }
  else if(!params.detector.empty() && params.descriptor.empty())
  {
    params.descriptor = params.detector;
  }

  if(!params.detector.empty() &&
     cv::FeatureDetector::create(params.detector).empty())
  {
    std::cout << "Unknown detector: " << params.detector << std::endl;
    exit(-3);
  }

  if(!params.descriptor.empty() &&
     cv::DescriptorExtractor::create(params.descriptor).empty())
  {
    std::cout << "Unknown descriptor: " << params.descriptor << std::endl;
    exit(-3);
  }
  else
  {
    params.binary_feature =
        params.descriptor == "ORB" ||
        params.descriptor == "FREAK" ||
        params.descriptor == "BRIEF" ||
        params.descriptor == "BRISK";
  }

  // sets output file
  {
    std::stringstream ss;
    ss << params.detector << "_" << params.descriptor
       << "_k" << params.k << "L" << params.L << ".voc.gz";
    params.output_filename = params.output_dir + "/" + ss.str();
  }

  // check that output file is writable
  {
    std::fstream f(params.output_filename.c_str(), std::ios::out);
    if(!f.is_open())
    {
      std::cout << "Output filename is not writable: " << params.output_filename
                   << std::endl;
      exit(-4);
    }
    else
    {
      DUtils::FileFunctions::RmFile(params.output_filename.c_str());
    }
  }
}

void getImageNames(const tParameters& params, std::vector<std::string>& files)
{
  files.clear();

  // check if input dir is a file with filenames
  std::fstream f(params.input_dir.c_str(), std::ios::in);
  if(f.is_open())
  {
    std::string filename;
    while(std::getline(f, filename))
    {
      if(!filename.empty()) files.push_back(filename);
    }
  }
  else if(DUtils::FileFunctions::DirExists(params.input_dir.c_str()))
  {
    // is a directory
    const std::vector<std::string> exts =
      {".jpg", ".png", ".pnm", ".pgm", ".bmp", ".gif"};

    for(auto ext : exts)
    {
      std::vector<std::string> subfiles =
          DUtils::FileFunctions::Dir(params.input_dir.c_str(), ext.c_str(),
                                     false, false);
      files.insert(files.end(), subfiles.begin(), subfiles.end());
    }

    std::sort(files.begin(), files.end());
  }
}
