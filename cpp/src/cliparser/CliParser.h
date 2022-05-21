#ifndef CLIPARSER_H
#define CLIPARSER_H

#include <algorithm>
#include <iomanip>
#include <ios>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/** A simple command-line argument parser
 *
 * CliParser is a simple C++ class the can parser arguments.
 *
 * \code
 *   int main(int argc, char** argv) {
 *     CliParser parser;
 *     parser.set_program_description("Add some elements to an input file");
 *     parser.add_flag("-v", "--verbose");
 *     parser.add_argflag("-N");
 *     parser.add_argflag("-o", "--output");
 *     parser.add_positional("input");
 *     parser.set_required("input");
 *     parser.set_required("--output");
 *     parser.set_description("-N", "number of elements");
 *     parser.set_description("--output", "output file location");
 *     parser.set_description("input", "file to read");
 *
 *     // use parser.parse_with_exceptions() if you want to use exceptions
 *     // rather than the parser calling exit().
 *     parser.parse(argc, argv); // will call exit() with --help or with error
 *
 *     bool verbose = parser.has("-v");
 *     int N = parser.get("-N", 10); // use 10 as default
 *     auto output = parser["--output"];
 *     auto input = parser["input"];
 *     std::vector<std::string> remaining = parser.remaining();
 *
 *     // the rest of main...
 *   }
 * \endcode
 */
class CliParser {
public:
  using string = std::string;
  template <typename T> using vector = std::vector<T>;
  template <typename T> using set = std::unordered_set<T>;

  class ParseError : public std::invalid_argument {
  public:
    using std::invalid_argument::invalid_argument; // use the same constructor
  };

  class MissingRequiredError : public ParseError {
  public:
    using ParseError::ParseError; // use the same constructor
  };

  class MissingFlagValueError : public ParseError {
  public:
    using ParseError::ParseError; // use the same constructor
  };

  class HelpRequest : public std::exception {
  public:
    HelpRequest() {} // only the default constructor
  };

protected:
  struct Option {
    vector<string> variants;
    bool expects_arg;
    bool required;
    string description;
    Option(vector<string> &&_variants, bool _expects_arg)
      : variants(std::move(_variants))
      , expects_arg(_expects_arg)
      , required(false)
    {}

    struct Less {
      bool operator()(std::shared_ptr<Option> a, std::shared_ptr<Option> b) {
        return lstrip_dashes(a->variants[0]) < lstrip_dashes(b->variants[0]);
      }
      bool operator()(const std::shared_ptr<Option> &a,
                      const std::shared_ptr<Option> &b) const
      {
        return lstrip_dashes(a->variants[0]) < lstrip_dashes(b->variants[0]);
      }
    };
  };

  struct PositionArg {
    string name;
    bool required;
    string description;
    PositionArg(string _name, bool _required)
      : name(_name), required(_required), description() {}
  };

  using OpPtr = std::shared_ptr<Option>;
  using PosPtr = std::shared_ptr<PositionArg>;
  using OptionMap = std::unordered_map<string, OpPtr>;
  using ParseMap = std::unordered_map<string, string*>;

public:
  CliParser() {
    add_flag("-h", "--help");
    set_description("--help", "Print this help and exit");
  }
  CliParser(const CliParser& other) = delete;
  CliParser(CliParser &&other) = default;
  virtual ~CliParser() = default;

  const vector<string> &args() const { return _args; }
  const string &program_name() const { return _args.at(0); }
  const vector<string> &remaining() const { return _remaining; }

  /// flag is just like "--help" needing no additional argument
  template <typename ... Args> void add_flag(string flag, Args ... flags) {
    vector<string> all;
    add_flag(all, false, flag, flags ...);
  }

  /// argflag is a flag with an argument, e.g., "--outfile out.txt"
  template <typename ... Args> void add_argflag(string flag, Args ... flags) {
    vector<string> all;
    add_flag(all, true, flag, flags ...);
  }

  /// next positional argument that is not part of a flag
  void add_positional(string name) {
    if (_recognized.count(name) > 0) {
      throw ParseError("Already registered argument '" + name + "'");
    }
    _positional.emplace_back(std::make_shared<PositionArg>(name, false));
    _recognized.emplace(std::move(name));
  }

  void set_program_description(const string &desc) {
    _prog_description = desc;
  }

  // set the description of the given flag or positional argument
  void set_description(const string &name, const string &desc) {
    if (_optionmap.find(name) != _optionmap.end()) {
      _optionmap[name]->description = desc;
      return;
    }

    // check positional
    auto it = std::find_if(_positional.begin(), _positional.end(),
                           [&name](PosPtr p) { return p->name == name; });
    if (it != _positional.end()) {
      it->get()->description = desc;
      return;
    }

    throw ParseError("set_description(): Unrecognized option '" + name + "'");
  }

  /// set a flag or positional argument as required (all are optional by default)
  void set_required(const string &name) {
    // check flags
    if (_optionmap.find(name) != _optionmap.end()) {
      _optionmap[name]->required = true;
      return;
    }

    // check positional
    auto it = std::find_if(_positional.begin(), _positional.end(),
                           [&name](PosPtr p) { return p->name == name; });
    if (it != _positional.end()) {
      it->get()->required = true;
      return;
    }

    // unrecognized type
    throw ParseError("set_required(): Unrecognized option '" + name + "'");
  }

  /** return the string representation of the value for the parsed arg
   *
   * Throws a std::out_of_bounds exception if the flag or positional argument
   * was not found in parsing.
   *
   * This function is only valid after calling parse().
   *
   * For flags without arguments, the value will be the same as the flag name.
   * For flags that have multiple variants, you can use any of the variants.
   *
   * \code
   *   CliParser parser;
   *   parser.add_flag("-h", "--help");
   *   parser.add_argflag("-i", "--input");
   *   parser.add_positional("outfile");
   *   parser.set_required("outfile");
   *   parser.parse({"program-name", "-h", "--input", "file.txt", "out.txt"});
   *   std::cout << parser["-i"] << std::endl;
   *   std::cout << parser["--input"] << std::endl;
   *   std::cout << parser["-h"] << std::endl;
   *   std::cout << parser["--help"] << std::endl;
   *   std::cout << parser["outfile"] << std::endl;
   * \endcode
   * This will print
   *
   * \code
   *   file.txt
   *   file.txt
   *   -h
   *   --help
   *   out.txt
   * \endcode
   *
   * It is better for non-argument flags to call has() instead.  Also, it is
   * best to call has() before calling operator[]() for non-required flags
   * because it throws if the flag wasn't there, or you can use get() with a
   * default value when it is missing.
   */
  const string &operator[](const string &name) const {
    auto it = _parsed.find(name);
    if (it != _parsed.end()) {
      return *(it->second);
    }

    // check to see if we recognize name
    if (_recognized.count(name) > 0) {
      throw std::out_of_range("Not found in parsing: '" + name + "'");
    }

    throw ParseError("Unrecognized argument name: '" + name + "'");
  }

  /// returns true if the flag or positional argument was found in parse()
  bool has(const string &name) const {
    if (_parsed.find(name) != _parsed.end()) {
      return true;
    }

    if (_recognized.count(name) > 0) {
      return false;
    }

    // otherwise, it is unrecognized
    throw ParseError("Unrecognized argument name: '" + name + "'");
  }

  /** returns as a specific type the value obtained from operator[]().
   *  an optional default value can be given if the argument was not seen.
   *
   * @throws std::out_of_range if the argument was not specified on the command-line
   * @throws ParseError if the argument cannot be converted to type T
   */
  template <typename T> T get(const string &name) const {
    // Note: use std::boolalpha to treat "true" as true and "false" as false
    std::istringstream in(operator[](name));
    T val{};
    in >> std ::boolalpha >> val;
    if (in.fail()) {
      throw ParseError("cannot convert '" + operator[](name)
                                  + "' to the type '" + typeid(T).name());
    }
    return val;
  }

  /// instead of std::out_of_range, you can specify a default value
  template <typename T> T get(const string &name, T defaultval) const {
    if (has(name)) { return get<T>(name); }
    return defaultval;
  }

  std::string usage() {
    // get program name
    std::string progname;
    if (_args.size() > 0) {
      progname = _args[0];
    } else {
      progname = "<program-name>";
    }
    return usage(progname);
  }

  std::string usage(const std::string &progname) {
    std::ostringstream out;
    auto optional_ops = optional_flags();
    auto required_ops = required_flags();
    auto optional_pos = optional_positional();
    auto required_pos = required_positional();

    // brief usage section
    out << "Usage:\n"
           "  " << progname << " --help\n"
           "  " << progname << "\n";
    for (auto &op : optional_ops) { print_flag_usage(out, op); }
    for (auto &op : required_ops) { print_flag_usage(out, op); }
    for (auto &pos : _positional) {
      out << "    ";
      if (!pos->required) { out << "["; }
      out << "<" << pos->name << ">";
      if (!pos->required) { out << "]"; }
      out << "\n";
    }
    out << "\n";

    if (_prog_description.size() > 0) {
      out << "Description:\n"
             "  " << _prog_description << "\n"
             "\n";
    }

    // required positional section
    if (required_pos.size() > 0) {
      out << "Required Positional Arguments:\n";
      for (auto &pos : required_pos) {
        print_pos(out, pos);
      }
      out << "\n";
    }

    // optional positional section
    if (optional_pos.size() > 0) {
      out << "Optional Positional Arguments:\n";
      for (auto &pos : optional_pos) {
        print_pos(out, pos);
      }
      out << "\n";
    }

    // required flags section
    if (required_ops.size() > 0) {
      out << "Required Flags:\n";
      for (auto &op : required_ops) {
        print_flag(out, op);
      }
      out << "\n";
    }

    // optional flags section
    if (optional_ops.size() > 0) {
      out << "Optional Flags:\n";
      for (auto &op : optional_ops) {
        print_flag(out, op);
      }
      out << "\n";
    }

    return out.str();
  }

  /// parse command-line options
  void parse(int argc, const char * const * argv) {
    parse(vector<string>(argv, argv+argc));
  }
  void parse_with_exceptions(int argc, const char * const * argv) {
    parse_with_exceptions(vector<string>(argv, argv+argc));
  }

  void parse(vector<string> args) {
    try {
      parse_with_exceptions(std::move(args));
    } catch (HelpRequest&) {
      std::cout << usage();
      std::cout.flush();
      std::exit(0);
    } catch (ParseError &ex) {
      std::cerr << "ParseError: " << ex.what() << "\n";
      std::cerr.flush();
      std::exit(1);
    }
  }
  void parse_with_exceptions(vector<string> args) {
    _args = std::move(args);
    _parsed.clear();
    _remaining.clear();

    std::size_t pos = 0;
    for (auto it = _args.begin() + 1; it != _args.end(); it++) {
      // first check known flags
      auto &arg = *it;
      auto opit = _optionmap.find(arg);
      if (opit != _optionmap.end()) {
        OpPtr &op = opit->second;
        if (op->variants[0] == "-h") { // handle --help separately
          throw HelpRequest();
        }
        if (op->expects_arg) {
          it++;
          if (it == _args.end()) {
            throw MissingFlagValueError("Flag '" + arg +
                                        "' requires another parameter");
          }
        }
        for (auto &name : op->variants) {
          _parsed[name] = &(*it);
        }
      }
      // then check positional arguments
      else if (pos < _positional.size()) {
        _parsed[_positional[pos]->name] = &(*it);
        pos++;
      }
      // otherwise, it is a remaining argument
      else {
        _remaining.emplace_back(*it);
      }
    }

    // check for required flags
    for (auto &flag : required_flags()) {
      if (!has(flag->variants[0])) {
        throw MissingRequiredError("Missing required flag '" +
                                   flag->variants[0] + "'");
      }
    }

    // check for required positional
    for (auto &pos : required_positional()) {
      if (!has(pos->name)) {
        throw MissingRequiredError("Missing required positional argument '" +
                                   pos->name + "'");
      }
    }
  }

protected:

  void add_flag(vector<string> &variants, bool hasarg, string flag) {
    variants.emplace_back(std::move(flag));
    OpPtr op = std::make_shared<Option>(std::move(variants), hasarg);
    for (auto &name : op->variants) {
      if (_recognized.count(name) > 0) {
        throw ParseError("Already registered argument '" + name + "'");
      }
    }
    for (auto &name : op->variants) {
      _recognized.emplace(name);
      _optionmap[name] = op;
    }
  }

  template <typename ... Args>
  void add_flag(vector<string> &variants, bool hasarg, string flag, Args ... flags) {
    variants.emplace_back(std::move(flag));
    add_flag(variants, hasarg, flags ...);
  }

  std::set<OpPtr, Option::Less> optional_flags() const {
    std::set<OpPtr, Option::Less> flags;
    for (auto &kv : _optionmap) {
      if (!kv.second->required) {
        flags.emplace(kv.second);
      }
    }
    return flags;
  }

  std::set<OpPtr, Option::Less> required_flags() const {
    std::set<OpPtr, Option::Less> flags;
    for (auto &kv : _optionmap) {
      if (kv.second->required) {
        flags.emplace(kv.second);
      }
    }
    return flags;
  }

  vector<PosPtr> optional_positional() const {
    vector<PosPtr> positional;
    for (auto &pos : _positional) {
      if (!pos->required) {
        positional.emplace_back(pos);
      }
    }
    return positional;
  }

  vector<PosPtr> required_positional() const {
    vector<PosPtr> positional;
    for (auto &pos : _positional) {
      if (pos->required) {
        positional.emplace_back(pos);
      }
    }
    return positional;
  }

  static string lstrip_dashes(const std::string &val) {
    return val.substr(val.find_first_not_of('-'));
  }

  static std::ostream& print_flag_usage(std::ostream& out, const OpPtr &p) {
    // we handle --help separately
    if (p->variants[0] == "-h") {
      return out;
    }

    out << "    ";
    if (!p->required) { out << "["; }
    out << p->variants[0];
    if (p->expects_arg) {
      out << " <val>";
    }
    if (!p->required) { out << "]"; }
    out << "\n";
    return out;
  }

  static std::ostream& print_flag(std::ostream& out, const OpPtr &p) {
    std::ostringstream tmpout;
    string suffix (p->expects_arg ? " <val>" : "");
    tmpout << "  " << p->variants[0] << suffix;
    for (size_t i = 1; i < p->variants.size(); i++) {
      tmpout << ", " << p->variants[i] << suffix;
    }
    out << tmpout.str();
    if (p->description != "") {
      if (tmpout.str().size() > 14) {
        out << "\n" << string(16, ' ');
      } else {
        out << string(16 - tmpout.str().size(), ' ');
      }
      out << p->description;
    }
    out << "\n";
    return out;
  }

  static std::ostream& print_pos(std::ostream& out, const PosPtr &p) {
    out << "  " << p->name;
    if (p->description.size() > 0) {
      if (p->name.size() >= 14) {
        out << "\n" << string(16, ' ');
      } else {
        out << string(14 - p->name.size(), ' ');
      }
      out << p->description;
    }
    return out << "\n";
  }

protected:
  string _prog_description;
  vector<string> _args;
  vector<PosPtr> _positional;
  set<string> _recognized;
  OptionMap _optionmap;
  ParseMap _parsed;
  vector<string> _remaining;
};

#endif // CLIPARSER_H
