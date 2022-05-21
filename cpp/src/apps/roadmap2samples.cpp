#include <cliparser/CliParser.h>
#include <cpptoml/toml_conversions.h>
#include <csv/Csv.h>
#include <tendon/TendonRobot.h>
#include <util/json_io.h>
#include <util/time_function_call.h>

#include <3rdparty/nlohmann/json.hpp>

#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <fstream>
#include <iostream>
#include <string>

using nlohmann::json;
namespace bio = boost::iostreams;

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description("Convert roadmap json to samples csv");

  parser.add_positional("robot");
  parser.set_required("robot");
  parser.set_description("robot", "robot toml description");

  parser.add_positional("roadmap");
  parser.set_required("roadmap");
  parser.set_description("roadmap", "roadmap json file in any supported json format");

  parser.add_argflag("-o", "--output");
  parser.set_description("--output", "output csv file.  default is samples.csv");

  parser.add_flag("-f", "--fill-in-missing-tips");
  parser.set_description("--fill-in-missing-tips",
                      "If the roadmap is missing tip positions, run FK to\n"
      "                fill them in.  Otherwise, just put 'NaN' for missing\n"
      "                values.");
}

void write_header(csv::CsvWriter &writer, const tendon::TendonRobot &robot) {
  writer << "probfile" << "index";
  for (size_t i = 0; i < robot.tendons.size(); ++i) {
    writer << "tau_" + std::to_string(i+1);
  }
  if (robot.enable_rotation) {
    writer << "theta";
  }
  if (robot.enable_retraction) {
    writer << "s_start";
  }
  writer << "tip_x" << "tip_y" << "tip_z";
  //writer << "quat_w" << "quat_x" << "quat_y" << "quat_z";
  writer.new_row();
}

bool ends_with(const std::string &s, const std::string &end) {
  return end.length() <= s.length()
      && std::string::npos != s.find(end, s.length() - end.length());
}

template <typename ContainerType>
void populate_vertex_tips(const tendon::TendonRobot &robot,
                          ContainerType &vertices)
{
  std::vector<double> state;
  #pragma omp parallel for private(state)
  for (size_t i = 0; i < vertices.size(); ++i) {
    auto &vertex = vertices[i];
    if (!vertex.contains("tip_pos")) {
      vertex["state"].get_to(state);
      auto tip_pos = robot.forward_kinematics(state).back();
      vertex["tip_pos"] = {tip_pos[0], tip_pos[1], tip_pos[2]};
    }
  }
}

auto load_vertices_from_dat(const std::string &roadmap_file) {
  std::vector<json> vertices;
  auto fin = std::make_unique<std::ifstream>();
  std::unique_ptr<std::istream> in;

  if (ends_with(roadmap_file, ".gz")) {
    util::openfile_check(*fin, roadmap_file,
                         std::ios_base::in | std::ios_base::binary);
    auto in_stream = std::make_unique<bio::filtering_stream<bio::input>>();
    in_stream->push(bio::gzip_decompressor());
    in_stream->push(*fin);
    in.reset(in_stream.release()); // transfer ownership
  } else {
    util::openfile_check(*fin, roadmap_file);
    in.reset(fin.release()); // transfer ownership
  }

  // parse the whole input, populating the vertices and edges
  std::string line;
  #pragma omp parallel
  {
    #pragma omp single
    while(std::getline(*in, line)) {
      #pragma omp task firstprivate(line) shared(vertices)
      {
        nlohmann::json obj;
        {
          std::istringstream line_in(line);
          obj = util::read_json(line_in, util::JsonFormat::JSON);
        }

        // parse the vertex
        if (obj["type"] == "vertex") {
          #pragma omp critical
          vertices.emplace_back(std::move(obj));
        }
      } // end of omp task
    } // end of while
  } // end of omp parallel

  return vertices;
}

template <typename ContainerType>
void vertices2samples(
    const std::string &robot_file,
    ContainerType &vertices,
    const std::string &output_file,
    bool auto_fill)
{
  float time;

  std::cout << "Sorting vertices" << std::flush;
  util::time_function_call([&]() {
        std::sort(vertices.begin(), vertices.end(),
                  [](auto &a, auto &b) { return a["index"] < b["index"]; });
      }, time);
  std::cout << " - done (" << time << " secs)\n";

  std::cout << "Loading in robot from " << robot_file << std::flush;
  auto robot = util::time_function_call([&]() {
        return cpptoml::from_file<tendon::TendonRobot>(robot_file);
      }, time);
  std::cout << " - done (" << time << " secs)\n";

  // pre-populate tips in parallel if requested
  if (auto_fill) {
    std::cout << "Filling in missing tip positions" << std::flush;
    util::time_function_call([&]() { populate_vertex_tips(robot, vertices); }, time);
    std::cout << " - done (" << time << " secs)\n";
  }

  std::ofstream out;
  util::openfile_check(out, output_file);
  csv::CsvWriter writer(out);
  std::cout << "Writing to " << output_file << std::endl;

  write_header(writer, robot);

  auto write_row = [&writer, &robot_file, &robot, auto_fill](json &vertex) {
    writer << robot_file << vertex["index"].get<int>();
    for (auto &val : vertex["state"]) {
      writer << val.get<double>();
    }
    if (vertex.contains("tip_pos")) {
      for (auto &val : vertex["tip_pos"]) {
        writer << val.get<double>();
      }
    } else {
      writer << "NaN" << "NaN" << "NaN";
    }
    // TODO: handle quaternions properly
    //writer << 1 << 0 << 0 << 0; // dummy quaternion for now

    writer.new_row();
  };

  for (auto &vert : vertices) {
    write_row(vert);
  }
}

} // end of unnamed namespace

int main(int arg_count, char* arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  auto robot_file   = parser["robot"];
  auto roadmap_file = parser["roadmap"];
  auto output_file  = parser.get("--output", std::string("samples.csv"));
  auto auto_fill    = parser.has("--fill-in-missing-tips");

  float time;

  std::cout << "Loading in roadmap from " << roadmap_file << std::flush;
  if (ends_with(roadmap_file, ".dat.gz") || ends_with(roadmap_file, ".dat")) {
    auto dat_vertices = util::time_function_call([&]() {
          return load_vertices_from_dat(roadmap_file);
        }, time);
    std::cout << " - done (" << time << " secs)\n";
    vertices2samples(robot_file, dat_vertices, output_file, auto_fill);
  } else {
    auto json_roadmap = util::time_function_call([&]() {
          return util::read_json(roadmap_file);
        }, time);
    auto &json_vertices = json_roadmap["VoxelCachedLazyPRM_roadmap"]["vertices"];
    std::cout << " - done (" << time << " secs)\n";
    vertices2samples(robot_file, json_vertices, output_file, auto_fill);
  }

  return 0;
}
