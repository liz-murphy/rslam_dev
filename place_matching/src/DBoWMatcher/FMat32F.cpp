#include <place_matching/DBoWMatcher/DUtils/DUtils.h>
#include <place_matching/DBoWMatcher/FMat32F.h>

void FMat32F::meanValue(
    const std::vector<FMat32F::pDescriptor> &descriptors,
    FMat32F::TDescriptor &mean)
{
  if(descriptors.empty())
  {
    mean.release();
    return;
  }
  else if(descriptors.size() == 1)
  {
    mean = descriptors[0]->clone();
  }
  else
  {
    const int L = descriptors[0]->cols;
    const float s = static_cast<float>(descriptors.size());

    mean = cv::Mat::zeros(1, L, CV_32F);
    for(size_t i = 0; i < descriptors.size(); ++i)
      mean += *descriptors[i] / s;
  }
}

// --------------------------------------------------------------------------

double FMat32F::distance(const FMat32F::TDescriptor &a,
  const FMat32F::TDescriptor &b)
{
  const float *pa = a.ptr<float>();
  const float *pb = b.ptr<float>();

  double sqd = 0.;
  for(int i = 0; i < a.cols; i += 4, pa += 4, pb += 4)
  {
    sqd += (pa[0] - pb[0])*(pa[0] - pb[0]);
    sqd += (pa[1] - pb[1])*(pa[1] - pb[1]);
    sqd += (pa[2] - pb[2])*(pa[2] - pb[2]);
    sqd += (pa[3] - pb[3])*(pa[3] - pb[3]);
  }

  int remaining = a.cols % 4;
  if(remaining > 0)
  {
    pa = a.ptr<float>(a.cols - remaining);
    pb = b.ptr<float>(a.cols - remaining);

    switch(remaining)
    {
      case 3: sqd += (pa[2] - pb[2])*(pa[2] - pb[2]);
      case 2: sqd += (pa[1] - pb[1])*(pa[1] - pb[1]);
      case 1: sqd += (pa[0] - pb[0])*(pa[0] - pb[0]);
    }
  }

  return sqd;
}

// --------------------------------------------------------------------------

std::string FMat32F::toString(const FMat32F::TDescriptor &a)
{
  std::stringstream ss;
  const float *pa = a.ptr<float>();
  for(int i = 0; i < a.cols; ++i, ++pa)
  {
    ss << *pa << " ";
  }
  return ss.str();
}

// --------------------------------------------------------------------------

void FMat32F::fromString(FMat32F::TDescriptor &a, const std::string &s)
{
  std::vector<std::string> tokens;
  DUtils::StringFunctions::split(s, tokens);

  a.create(1, tokens.size(), CV_32F);
  float *p = a.ptr<float>();

  std::stringstream ss(s);
  for(size_t i = 0; i < tokens.size(); ++i, ++p)
  {
    ss >> *p;
  }
}

// --------------------------------------------------------------------------

void FMat32F::toMat32F(const std::vector<TDescriptor> &descriptors, cv::Mat &mat)
{
  if(descriptors.empty())
    mat.release();
  else
  {
    const size_t N = descriptors.size();
    const size_t L = descriptors[0].cols;
    mat.create(N, L, CV_32F);

    float *p = mat.ptr<float>();
    for(size_t i = 0; i < descriptors.size(); ++i, p += L)
      std::copy(descriptors[i].begin<float>(), descriptors[i].end<float>(), p);
  }
}

