#include <PclRefLIB.h>
using namespace pclref;

template<>
size_t pclDataLength<pcl::FPFHSignature33>()
{
  return 33;
}
template<>
size_t pclDataLength<pcl::Narf36>()
{
  return 36;
}
template<>
size_t pclDataLength<pcl::VFHSignature308>()
{
  return 308;
}
template<>
size_t pclDataLength<pcl::ShapeContext1980>()
{
  return 1980;
}
template<>
size_t pclDataLength<pcl::SHOT352>()
{
  return 352;
}
template<>
size_t pclDataLength<pcl::ESFSignature640>()
{
  return 640;
}
template<>
size_t pclDataLength<pcl::Histogram<153> >()
{
  return 153;
}
template<>
size_t pclDataLength<pcl::Histogram<10> >()
{
  return 10;
}
template<>
const float* pclData<pcl::FPFHSignature33>(const pcl::FPFHSignature33& x)
{
  return x.histogram;
}
template<>
const float* pclData<pcl::Narf36>(const pcl::Narf36& x)
{
  return x.descriptor;
}
template<>
const float* pclData<pcl::VFHSignature308>(const pcl::VFHSignature308& x)
{
  return x.histogram;
}
template<>
const float* pclData<pcl::ESFSignature640>(const pcl::ESFSignature640& x)
{
  return x.histogram;
}
template<>
const float* pclData<pcl::Histogram<153> >(const pcl::Histogram<153>& x)
{
  return x.histogram;
}
template<>
const float* pclData<pcl::Histogram<10> >(const pcl::Histogram<10>& x)
{
  return x.histogram;
}
template<>
const float* pclData<pcl::ShapeContext1980>(const pcl::ShapeContext1980& x)
{
  return x.descriptor;
}
template<>
const float* pclData<pcl::SHOT352>(const pcl::SHOT352& x)
{
  return x.descriptor;
}
template<>
float* pclData<pcl::FPFHSignature33>(pcl::FPFHSignature33& x)
{
  return x.histogram;
}
template<>
float* pclData<pcl::Narf36>(pcl::Narf36& x)
{
  return x.descriptor;
}
template<>
float* pclData<pcl::VFHSignature308>(pcl::VFHSignature308& x)
{
  return x.histogram;
}
template<>
float* pclData<pcl::ShapeContext1980>(pcl::ShapeContext1980& x)
{
  return x.descriptor;
}
template<>
float* pclData<pcl::SHOT352>(pcl::SHOT352& x)
{
  return x.descriptor;
}
template<>
float* pclData<pcl::ESFSignature640>(pcl::ESFSignature640& x)
{
  return x.histogram;
}
template<>
float* pclData<pcl::Histogram<153> >(pcl::Histogram<153>& x)
{
  return x.histogram;
}
template<>
float* pclData<pcl::Histogram<10> >(pcl::Histogram<10>& x)
{
  return x.histogram;
}

