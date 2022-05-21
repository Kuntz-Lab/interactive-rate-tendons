#include <cliparser/CliParser.h>
#include <collision/VoxelOctree.h>
#include <cpptoml/toml_conversions.h>
#include <util/json_io.h>
#include <util/openfile_check.h>

#include <3rdparty/nlohmann/json.hpp>

#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <cctype>    // for std::tolower()
#include <algorithm>
#include <sstream>

namespace {

void populate_parser (CliParser &parser) {
  parser.set_program_description(
      "Convert from one voxel image format to another.");

  parser.add_argflag("-f", "--to-format");
  parser.set_required("--to-format");
  parser.set_description("--to-format",
                      "Output format for each input voxel image.\n"
      "                Supports nrrd, toml, toml.gz, json, bson, cbor,\n"
      "                msgpack, and ubjson.\n"
      "                Will append the new file extension to the input file\n"
      "                name.");
  
  parser.add_positional("voxel_images");
  parser.set_required("voxel_images");
  parser.set_description("voxel_images",
                      "One or more voxel images.\n"
      "                Supports nrrd, toml, toml.gz, json, bson, cbor,\n"
      "                msgpack, and ubjson.");
}

struct ParsedArgs {
  std::vector<std::string> voxel_files;
  std::string extension;
};

ParsedArgs parse_args (CliParser &parser, int arg_count, char* arg_list[]) {
  parser.parse(arg_count, arg_list);
  ParsedArgs args;

  args.voxel_files.emplace_back(parser["voxel_images"]);
  auto &remaining = parser.remaining();
  args.voxel_files.insert(args.voxel_files.end(), remaining.begin(), remaining.end());

  args.extension = parser["--to-format"];

  return args;
}

} // end of unnamed namespace

int main (int arg_count, char* arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  auto args = parse_args(parser, arg_count, arg_list);

  for (const auto &fname : args.voxel_files) {
    std::cout << "reading " << fname << std::endl;
    auto outfile = fname + "." + args.extension;
    auto voxels = collision::VoxelOctree::from_file(fname);
    std::cout << "writing " << outfile << std::endl;
    if (args.extension == "nrrd") {
      voxels.to_nrrd(outfile);
    } else if (args.extension == "toml" || args.extension == "toml.gz") {
      cpptoml::to_file(voxels, outfile);
    } else {
      util::write_json(outfile, voxels.to_json());
    }
  }

  return 0;
}
