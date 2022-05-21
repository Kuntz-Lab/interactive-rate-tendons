#include "cpptoml/toml_conversions.h"
#include <util/openfile_check.h>

#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <fstream>
#include <string>

namespace cpptoml {

void act_fout(const std::string &fname,
              const std::function<void(std::ostream&)> &f)
{
  namespace bio = boost::iostreams;
  std::ofstream fout;
  if (util::endswith(fname, ".gz")) { // use gzip compression
    util::openfile_check(fout, fname, std::ios_base::out | std::ios_base::binary);
    bio::filtering_stream <bio::output> out;
    out.push(bio::gzip_compressor());
    out.push(fout);
    f(out);
  } else {
    util::openfile_check(fout, fname);
    f(fout);
  }
}

void act_fin(const std::string &fname,
             const std::function<void(std::istream&)> &f)
{
  namespace bio = boost::iostreams;
  std::ifstream fin;
  if (util::endswith(fname, ".gz")) { // use gzip compression
    util::openfile_check(fin, fname, std::ios_base::in | std::ios_base::binary);
    bio::filtering_stream<bio::input> in;
    in.push(bio::gzip_decompressor());
    in.push(fin);
    f(in);
  } else {
    util::openfile_check(fin, fname);
    f(fin);
  }
}

} // end of namespace cpptoml
