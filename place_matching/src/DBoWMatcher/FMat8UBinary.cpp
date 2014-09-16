#include <DUtils.h>
#include "FMat8UBinary.h"

void FMat8UBinary::meanValue(
    const std::vector<FMat8UBinary::pDescriptor> &descriptors,
    FMat8UBinary::TDescriptor &mean)
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
    std::vector<int> sum(L * 8, 0);

    for(size_t i = 0; i < descriptors.size(); ++i)
    {
      const cv::Mat &d = *descriptors[i];
      const unsigned char *p = d.ptr<unsigned char>();

      for(int j = 0; j < d.cols; ++j, ++p)
      {
        if(*p & (1 << 7)) ++sum[ j*8     ];
        if(*p & (1 << 6)) ++sum[ j*8 + 1 ];
        if(*p & (1 << 5)) ++sum[ j*8 + 2 ];
        if(*p & (1 << 4)) ++sum[ j*8 + 3 ];
        if(*p & (1 << 3)) ++sum[ j*8 + 4 ];
        if(*p & (1 << 2)) ++sum[ j*8 + 5 ];
        if(*p & (1 << 1)) ++sum[ j*8 + 6 ];
        if(*p & (1))      ++sum[ j*8 + 7 ];
      }
    }

    mean = cv::Mat::zeros(1, L, CV_8U);
    unsigned char *p = mean.ptr<unsigned char>();

    const int N2 = (int)descriptors.size() / 2 + descriptors.size() % 2;
    for(size_t i = 0; i < sum.size(); ++i)
    {
      if(sum[i] >= N2)
      {
        // set bit
        *p |= 1 << (7 - (i % 8));
      }

      if(i % 8 == 7) ++p;
    }
  }
}

// --------------------------------------------------------------------------

double FMat8UBinary::distance(const FMat8UBinary::TDescriptor &a,
  const FMat8UBinary::TDescriptor &b)
{
  #if 0
  // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetKernighan
  // NOTE: this implementation assumes that sizeof(unsigned int) == 32 bits,
  // and that a.cols (CV_8U) % sizeof(unsigned int) == 0

  const unsigned int *pa, *pb;
  pa = a.ptr<unsigned int>(); // a & b are actually CV_8U
  pb = b.ptr<unsigned int>();

  int v, ret = 0;
  for(size_t i = 0; i < a.cols / sizeof(unsigned int); ++i, ++pa, ++pb)
  {
    v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);                    // reuse input as temporary
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);     // temp
    ret += ((v + (v >> 4) & 0xF0F0F0F) * 0x1010101) >> 24; // count
  }

  return ret;

  #else

  // portable approach
  const unsigned char *pa, *pb;
  pa = a.ptr<unsigned char>();
  pb = b.ptr<unsigned char>();

  int ret = 0;
  for(int i = 0; i < a.cols; ++i, ++pa, ++pb)
  {
    ret += DUtils::LUT::ones8bits[ *pa ^ *pb ];
  }

  return ret;
  #endif
}

// --------------------------------------------------------------------------

std::string FMat8UBinary::toString(const FMat8UBinary::TDescriptor &a)
{
  std::stringstream ss;
  const unsigned char *p = a.ptr<unsigned char>();

  for(int i = 0; i < a.cols; ++i, ++p)
  {
    ss << (int)*p << " ";
  }

  return ss.str();
}

// --------------------------------------------------------------------------

void FMat8UBinary::fromString(FMat8UBinary::TDescriptor &a, const std::string &s)
{
  std::vector<std::string> tokens;
  DUtils::StringFunctions::split(s, tokens);

  a.create(1, tokens.size(), CV_8U);
  unsigned char *p = a.ptr<unsigned char>();

  std::stringstream ss(s);
  for(size_t i = 0; i < tokens.size(); ++i, ++p)
  {
    int n;
    ss >> n;

    if(!ss.fail())
      *p = (unsigned char)n;
  }
}

// --------------------------------------------------------------------------

void FMat8UBinary::toMat32F(const std::vector<TDescriptor> &descriptors,
  cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }

  const size_t N = descriptors.size();

  mat.create(N, descriptors[0].cols*8, CV_32F);
  float *p = mat.ptr<float>();

  for(size_t i = 0; i < N; ++i)
  {
    const int C = descriptors[i].cols;
    const unsigned char *desc = descriptors[i].ptr<unsigned char>();

    for(int j = 0; j < C; ++j, p += 8)
    {
      p[0] = (desc[j] & (1 << 7) ? 1 : 0);
      p[1] = (desc[j] & (1 << 6) ? 1 : 0);
      p[2] = (desc[j] & (1 << 5) ? 1 : 0);
      p[3] = (desc[j] & (1 << 4) ? 1 : 0);
      p[4] = (desc[j] & (1 << 3) ? 1 : 0);
      p[5] = (desc[j] & (1 << 2) ? 1 : 0);
      p[6] = (desc[j] & (1 << 1) ? 1 : 0);
      p[7] = desc[j] & (1);
    }
  }
}

// --------------------------------------------------------------------------
/*
void FMat8UBinary::toMat32F(const cv::Mat &descriptors, cv::Mat &mat)
{

  descriptors.convertTo(mat, CV_32F);
  return;

  if(descriptors.empty())
  {
    mat.release();
    return;
  }

  const int N = descriptors.rows;
  const int C = descriptors.cols;

  mat.create(N, descriptors[0].cols*8, CV_32F);
  float *p = mat.ptr<float>(); // p[i] == 1 or 0

  const unsigned char *desc = descriptors.ptr<unsigned char>();

  for(int i = 0; i < N; ++i, desc += C)
  {
    for(int j = 0; j < C; ++j, p += 8)
    {
      p[0] = (desc[j] & (1 << 7) ? 1 : 0);
      p[1] = (desc[j] & (1 << 6) ? 1 : 0);
      p[2] = (desc[j] & (1 << 5) ? 1 : 0);
      p[3] = (desc[j] & (1 << 4) ? 1 : 0);
      p[4] = (desc[j] & (1 << 3) ? 1 : 0);
      p[5] = (desc[j] & (1 << 2) ? 1 : 0);
      p[6] = (desc[j] & (1 << 1) ? 1 : 0);
      p[7] = desc[j] & (1);
    }
  }
}
*/

// --------------------------------------------------------------------------

/*
void FMat8UBinary::toMat8U(const std::vector<TDescriptor> &descriptors,
  cv::Mat &mat)
{
  mat.create(descriptors.size(), FMat8UBinary::L, CV_8U);

  unsigned char *p = mat.ptr<unsigned char>();

  for(size_t i = 0; i < descriptors.size(); ++i, p += FMat8UBinary::L)
  {
    const unsigned char *d = descriptors[i].ptr<unsigned char>();
    std::copy(d, d + FMat8UBinary::L, p);
  }
}
*/
